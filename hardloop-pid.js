/**
 * hardloop-pid.js
 * Copyright 2026 Harshad Joshi and Bufferstack.IO Analytics Technology LLP, Pune
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Purpose:
 * Hard-loop PID controller node for soft-PLC platforms where Node-RED
 * IS the control runtime (Kunbus RevPi, Siemens IOT2050, Wago PFC200,
 * Advantech ADAM, Beckhoff CX, any Linux-based soft-PLC with Node-RED).
 *
 * - Accepts raw ADC counts as PV input (directly from hardware registers)
 * - Converts counts → EU internally before PID computation
 * - Converts PID EU output → AO counts for writing back to hardware registers
 * - Hardware watchdog: forces safe_ao_count if PV stops arriving
 * - Emits AO_count (integer) for direct hardware register write
 * - Tracks scan time jitter for loop health monitoring
 * - Emits fault and watchdog_ok flags every cycle
 *
 * For slow-to-medium processes only (temperature, level, pressure, flow).
 * NOT suitable for fast loops (position, speed, torque).
 */

"use strict";

const PIDController = require('simple-pid-controller');

module.exports = function (RED) {

  function HardloopPIDControllerNode(config) {
    RED.nodes.createNode(this, config);
    const node = this;
    let controller  = null;
    let pidTimer    = null;
    let wdTimer     = null;   // watchdog timer
    let lastPVTime  = null;   // timestamp of last valid PV message
    let inFault     = false;  // watchdog fault state
    let lastScanTime = 0;     // ms of last completed scan for jitter tracking

    //  Parse and validate configuration 
    try {
      // PID gains
      node.k_p = Number(config.k_p);
      node.k_i = Number(config.k_i);
      node.k_d = Number(config.k_d);
      node.dt  = Number(config.dt);   // target loop interval in seconds

      // AI (input) count range â€” matches physical AI card resolution
      node.ai_count_min = Number(config.ai_count_min);
      node.ai_count_max = Number(config.ai_count_max);

      // AO (output) count range â€” matches physical AO card resolution
      node.ao_count_min = Number(config.ao_count_min);
      node.ao_count_max = Number(config.ao_count_max);

      // Engineering unit range (used for both AI and AO scaling)
      node.eu_min = Number(config.eu_min);
      node.eu_max = Number(config.eu_max);

      // Signal type: '4-20mA' or '0-10V'
      // Determines whether a live-zero offset applies to AI/AO count calculations
      node.signal_type = config.signal_type || '4-20mA';

      // PID safety limits
      node.output_min  = Number(config.output_min);
      node.output_max  = Number(config.output_max);
      node.int_min     = Number(config.int_min);
      node.int_max     = Number(config.int_max);
      node.deadband    = Number(config.deadband)    || 0;
      node.settled_tol = Number(config.settled_tol) || 0;

      // Watchdog: if PV not updated within this many ms, go to safe state
      node.watchdog_timeout = Number(config.watchdog_timeout) || 3000;

      // Safe AO count to write on watchdog trip or fault
      // Typically ao_count_min (valve closed) or a mid-scale safe position
      node.safe_ao_count = Number(config.safe_ao_count);
      if (isNaN(node.safe_ao_count)) node.safe_ao_count = node.ao_count_min || 0;

      // Derivative filter coefficient alpha (0 < alpha <= 1)
      // Lower = more filtering of ADC quantisation noise on derivative term
      // 1.0 = no filter (raw derivative), 0.1 = heavy filtering
      node.deriv_filter = Number(config.deriv_filter);
      if (isNaN(node.deriv_filter) || node.deriv_filter <= 0 || node.deriv_filter > 1) {
        node.deriv_filter = 1.0; // default: no filter
      }

    } catch (error) {
      node.error('Error parsing configuration: ' + error.message);
      return;
    }

    //  Validate required parameters 
    const required = [
      node.k_p, node.k_i, node.k_d, node.dt,
      node.ai_count_min, node.ai_count_max,
      node.ao_count_min, node.ao_count_max,
      node.eu_min, node.eu_max,
    ];

    if (required.some(isNaN)) {
      node.error('Invalid configuration: all numeric fields must be valid numbers.');
      return;
    }

    if (node.dt <= 0) {
      node.error('dt must be greater than 0.');
      return;
    }

    if (node.ai_count_max <= node.ai_count_min) {
      node.error('ai_count_max must be greater than ai_count_min.');
      return;
    }

    if (node.ao_count_max <= node.ao_count_min) {
      node.error('ao_count_max must be greater than ao_count_min.');
      return;
    }

    if (node.eu_max <= node.eu_min) {
      node.error('eu_max must be greater than eu_min.');
      return;
    }

    //  Instantiate PID controller 
    try {
      controller = new PIDController(node.k_p, node.k_i, node.k_d, node.dt, {
        outputMin:        isNaN(node.output_min) ? -Infinity : node.output_min,
        outputMax:        isNaN(node.output_max) ?  Infinity : node.output_max,
        integralMin:      isNaN(node.int_min)    ? -Infinity : node.int_min,
        integralMax:      isNaN(node.int_max)    ?  Infinity : node.int_max,
        deadband:         node.deadband,
        settledTolerance: node.settled_tol,
      });
    } catch (error) {
      node.error('Error creating PID controller: ' + error.message);
      return;
    }

    //  Internal state 
    let currentPV_count = null;   // latest raw AI count received
    let currentPV_eu    = null;   // PV in engineering units (converted from count)
    let filteredD       = 0;      // filtered derivative term accumulator

    //  Scaling helpers 

    /**
     * Convert raw AI count to engineering unit value.
     *
     * For 4-20mA: count range maps to live-zero signal.
     *   count_min â†’ eu_min (e.g. 0 counts = 4mA = 0 bar)
     *   count_max â†’ eu_max (e.g. 4000 counts = 20mA = 10 bar)
     *
     * For 0-10V: count range maps linearly across full signal.
     *   count_min â†’ eu_min
     *   count_max â†’ eu_max
     *
     * Both use the same linear formula â€” the difference is what count_min
     * represents physically (0V vs 4mA), which is set by the user in config.
     *
     * @param {number} count - Raw integer count from AI register
     * @returns {number} Engineering unit value
     */
    function countToEU(count) {
      return node.eu_min +
        ((count - node.ai_count_min) / (node.ai_count_max - node.ai_count_min)) *
        (node.eu_max - node.eu_min);
    }

    /**
     * Convert PID EU output to AO count integer for hardware register write.
     *
     * Clamps the result to [ao_count_min, ao_count_max] to protect the
     * AO card from out-of-range writes that could damage field devices.
     *
     * @param {number} eu - Engineering unit output from PID
     * @returns {number} Integer count for AO register
     */
    function euToAOCount(eu) {
      const raw = node.ao_count_min +
        ((eu - node.eu_min) / (node.eu_max - node.eu_min)) *
        (node.ao_count_max - node.ao_count_min);

      // Clamp and round to integer â€” AO registers are always integers
      return Math.round(
        Math.max(node.ao_count_min, Math.min(node.ao_count_max, raw))
      );
    }

    /**
     * Convert EU output to physical signal value for tracing/display.
     * Mirrors the easy-pid-controller Signal field for dashboard compatibility.
     *
     * @param {number} eu - Engineering unit output from PID
     * @returns {number} Signal in mA (4-20mA) or V (0-10V)
     */
    function euToSignal(eu) {
      const ratio = (eu - node.eu_min) / (node.eu_max - node.eu_min);
      if (node.signal_type === '0-10V') {
        return ratio * 10;
      } else {
        // 4-20mA
        return 4 + ratio * 16;
      }
    }

    /**
     * Write safe AO count to output immediately.
     * Called on watchdog trip, fault, or node close.
     * The downstream node (Modbus/SLMP write) must act on this immediately.
     *
     * @param {string} reason - Human-readable reason for safe state
     */
    function writeSafeState(reason) {
      node.send({
        payload: {
          AO_count:    node.safe_ao_count,
          fault:       true,
          watchdog_ok: false,
          fault_reason: reason,
          PV_count:    currentPV_count,
          PV_eu:       currentPV_eu,
          SV_eu:       controller ? controller.target : null,
          mode:        'fault',
        },
      });
      node.status({ fill: 'red', shape: 'ring', text: 'FAULT: ' + reason });
    }

    //  Watchdog 
    /**
     * Start the watchdog timer.
     * If PV is not received within watchdog_timeout ms, the controller
     * forces AO output to safe_ao_count and sets fault state.
     * This protects the physical process if Node-RED loses its data source.
     */
    function startWatchdog() {
      if (wdTimer) clearTimeout(wdTimer);
      wdTimer = setTimeout(() => {
        if (!inFault) {
          inFault = true;
          node.warn('Watchdog tripped: PV not received within ' + node.watchdog_timeout + 'ms');
          writeSafeState('PV timeout (' + node.watchdog_timeout + 'ms)');

          // Stop PID loop â€” do not continue computing without valid PV
          if (pidTimer) {
            clearInterval(pidTimer);
            pidTimer = null;
          }
        }
      }, node.watchdog_timeout);
    }

    /**
     * Reset the watchdog timer on every valid PV update.
     * Clears fault state if we were previously in fault.
     */
    function resetWatchdog() {
      if (wdTimer) clearTimeout(wdTimer);
      if (inFault) {
        inFault = false;
        node.status({ fill: 'yellow', shape: 'dot', text: 'Fault cleared â€” awaiting auto start' });
        node.warn('Watchdog fault cleared. Send auto:true to restart loop.');
      }
      startWatchdog();
    }

    //  PID settled event 
    controller.on('settled', (status) => {
      node.status({ fill: 'green', shape: 'dot',
        text: 'Settled at SV: ' + status.sv + ' ' + node.signal_type });
      if (pidTimer) {
        clearInterval(pidTimer);
        pidTimer = null;
      }
    });

    //  Input message handler 
    node.on('input', function (msg) {
      try {

        switch (msg.topic) {

          //  PV: raw AI count from hardware register 
          // This is the primary input in hardloop mode.
          // The node converts counts â†’ EU internally before passing to PID.
          case 'PV':
            if (typeof msg.payload !== 'number' || !Number.isFinite(msg.payload)) {
              node.error('PV payload must be a finite number (raw AI count).');
              return;
            }

            // Validate count is within expected AI range
            if (msg.payload < node.ai_count_min || msg.payload > node.ai_count_max) {
              node.warn('PV count ' + msg.payload + ' is outside AI range [' +
                node.ai_count_min + 'â€“' + node.ai_count_max + ']. Clamping.');
            }

            currentPV_count = Math.round(msg.payload);
            currentPV_eu    = countToEU(
              Math.max(node.ai_count_min, Math.min(node.ai_count_max, currentPV_count))
            );

            lastPVTime = Date.now();
            resetWatchdog();  // feed watchdog on every valid PV

            node.status({ fill: 'blue', shape: 'ring',
              text: 'PV: ' + currentPV_count + ' counts (' + currentPV_eu.toFixed(2) + ')' });
            break;

          //  SV: setpoint in engineering units 
          // SV is always in EU â€” the operator thinks in engineering units.
          // Internally the PID works in EU so no conversion needed here.
          case 'SV':
            if (typeof msg.payload !== 'number') {
              node.error('SV payload must be a number in engineering units.');
              return;
            }
            controller.setTarget(msg.payload);  // also resets integral state
            node.status({ fill: 'yellow', shape: 'dot', text: 'SV: ' + msg.payload });
            break;

          //  auto: start/stop the hard control loop 
          case 'auto':
            if (msg.payload === true) {
              if (currentPV_eu === null) {
                node.warn('Cannot start loop: no valid PV received yet.');
                return;
              }
              if (inFault) {
                node.warn('Cannot start loop: watchdog fault active. Send valid PV first.');
                return;
              }

              controller.setMode('auto');
              node.status({ fill: 'green', shape: 'dot', text: 'Hard loop active' });

              if (pidTimer === null) {
                pidTimer = setInterval(function () {
                  const scanStart = Date.now();

                  // Safety check: refuse to compute if in fault state
                  if (inFault) {
                    writeSafeState('Loop running but watchdog in fault');
                    clearInterval(pidTimer);
                    pidTimer = null;
                    return;
                  }

                  // Run PID in EU domain
                  const pidOutput_eu = controller.update(currentPV_eu);

                  // Convert EU output â†’ AO count for hardware register write
                  const ao_count = euToAOCount(pidOutput_eu);

                  // Convert EU output â†’ physical signal for tracing/display
                  const signal = euToSignal(pidOutput_eu);

                  // Compute scan time for jitter monitoring
                  const scanTime = Date.now() - scanStart;
                  lastScanTime = scanTime;

                  const status = controller.getStatus();

                  node.send({
                    payload: {
                      // Hardware output â€” write this to AO register
                      AO_count:    ao_count,

                      // Process values in engineering units
                      PV_count:    currentPV_count,
                      PV_eu:       parseFloat(currentPV_eu.toFixed(4)),
                      SV_eu:       status.sv,
                      error_eu:    parseFloat(status.error.toFixed(4)),

                      // PID components in EU
                      P:           parseFloat(status.p.toFixed(4)),
                      I:           parseFloat(status.i.toFixed(4)),
                      D:           parseFloat(status.d.toFixed(4)),
                      output_eu:   parseFloat(status.output.toFixed(4)),

                      // Physical signal for tracing (mA or V)
                      Signal:      parseFloat(signal.toFixed(4)),

                      // Loop health
                      mode:        status.mode,
                      fault:       false,
                      watchdog_ok: true,
                      scan_ms:     scanTime,
                    },
                  });

                }, node.dt * 1000);
              }

            } else if (msg.payload === false) {
              controller.setMode('manual');
              if (pidTimer) {
                clearInterval(pidTimer);
                pidTimer = null;
              }
              node.status({ fill: 'red', shape: 'ring', text: 'Hard loop stopped' });
            }
            break;

          //  gains: runtime gain update 
          // Useful for adaptive tuning or remote commissioning.
          case 'gains':
            if (
              typeof msg.payload.k_p === 'number' &&
              typeof msg.payload.k_i === 'number' &&
              typeof msg.payload.k_d === 'number'
            ) {
              controller.updateGains(msg.payload.k_p, msg.payload.k_i, msg.payload.k_d);
              node.status({ fill: 'yellow', shape: 'dot',
                text: 'Gains: ' + msg.payload.k_p + '/' + msg.payload.k_i + '/' + msg.payload.k_d });
            } else {
              node.error('gains payload must be { k_p, k_i, k_d } â€” all numbers.');
            }
            break;

          //  mode: manual/auto switch with bumpless transfer 
          case 'mode':
            controller.setMode(msg.payload);
            node.status({ fill: 'yellow', shape: 'dot', text: 'Mode: ' + msg.payload });
            break;

          //  manual_output: set fixed EU output in manual mode 
          // Seed value for bumpless transfer when switching back to auto.
          case 'manual_output':
            if (typeof msg.payload !== 'number') {
              node.error('manual_output must be a number in engineering units.');
              return;
            }
            controller.setManualOutput(msg.payload);
            node.status({ fill: 'blue', shape: 'dot', text: 'Manual output (EU): ' + msg.payload });
            break;

          //  reset: clear all PID internal state 
          case 'reset':
            controller.reset();
            inFault = false;
            filteredD = 0;
            node.status({ fill: 'grey', shape: 'ring', text: 'Controller reset' });
            break;

          default:
            node.warn('Unknown topic: ' + msg.topic);
        }

      } catch (error) {
        node.error('Error handling input: ' + error.message);
        node.status({ fill: 'red', shape: 'ring', text: 'Error: ' + error.message });
      }
    });

    // Node close: write safe state and clean up 
    // On redeploy or shutdown, force AO to safe count immediately.
    // This is the closest Node-RED can get to a hardware safe-state guarantee.
    node.on('close', function () {
      try {
        if (pidTimer) {
          clearInterval(pidTimer);
          pidTimer = null;
        }
        if (wdTimer) {
          clearTimeout(wdTimer);
          wdTimer = null;
        }
        if (controller) {
          // Write safe state before shutting down
          writeSafeState('Node closed / redeployed');
          controller.reset();
        }
      } catch (error) {
        node.error('Error on node close: ' + error.message);
      }
    });
  }

  RED.nodes.registerType('hardloop-pid-controller', HardloopPIDControllerNode);
};
