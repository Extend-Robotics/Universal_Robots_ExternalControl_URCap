// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-18
 *
 */
//----------------------------------------------------------------------

package com.fzi.externalcontrol.impl;

import com.ur.urcap.api.contribution.InstallationNodeContribution;
import com.ur.urcap.api.contribution.installation.InstallationAPIProvider;
import com.ur.urcap.api.domain.data.DataModel;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputCallback;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputFactory;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardNumberInput;

public class ExternalControlInstallationNodeContribution implements InstallationNodeContribution {
  private static final String HOST_IP = "host_ip";
  private static final String PORT_NR = "port_nr";
  private static final String NAME = "name";
  private static final String DEFAULT_IP = "192.168.56.1";
  private static final Integer DEFAULT_PORT = 50002;
  private static final String DEFAULT_NAME = DEFAULT_IP;
  private DataModel model;
  private final ExternalControlInstallationNodeView view;
  private final KeyboardInputFactory keyboardFactory;
  private RequestProgram sender;
  private boolean programRequested;    
  private String control_header;     //NSChange Added new Variable to hold the string 


  public ExternalControlInstallationNodeContribution(InstallationAPIProvider apiProvider,
      ExternalControlInstallationNodeView view, DataModel model) {
    this.keyboardFactory =
        apiProvider.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory();
    this.model = model;
    this.view = view;
  }

  @Override
  public void openView() {}

  @Override
  public void closeView() {}

  public boolean isDefined() {
    return !getHostIP().isEmpty();
  }
  //Add the Installation method here
  @Override
  public void generateScript(ScriptWriter writer) {
    programRequested = false;
    //NSChange  Start 

    writer.appendLine("steptime = get_steptime()");

    writer.appendLine("MULT_jointstate = 1000000");

    writer.appendLine("#Constants");
    writer.appendLine("SERVO_UNINITIALIZED = -1");
    writer.appendLine("SERVO_IDLE = 0");
    writer.appendLine("SERVO_RUNNING = 1");

    writer.appendLine("MODE_STOPPED = -2");
    writer.appendLine("MODE_UNINITIALIZED = -1");
    writer.appendLine("MODE_IDLE = 0");
    writer.appendLine("MODE_SERVOJ = 1");
    writer.appendLine("MODE_SPEEDJ = 2");

    writer.appendLine("#Global variables are also showed in the Teach pendants variable list");
    writer.appendLine("global er_cmd_servo_state = SERVO_UNINITIALIZED");
    writer.appendLine("global er_cmd_servo_qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]");
    writer.appendLine("global er_cmd_servo_q = get_actual_joint_positions()");
    writer.appendLine("global er_cmd_servo_q_last = get_actual_joint_positions()");
    writer.appendLine("global er_extrapolate_count = 0");
    writer.appendLine("global er_extrapolate_max_count = 0");
    writer.appendLine("global er_control_mode = MODE_UNINITIALIZED");
    writer.appendLine("cmd_speedj_active = True");

    writer.appendLine("def set_servo_setpoint(q):");
    writer.appendLine("  er_cmd_servo_state = SERVO_RUNNING");
    writer.appendLine("  er_cmd_servo_q_last = er_cmd_servo_q");
    writer.appendLine("  er_cmd_servo_q = q");
    writer.appendLine("end");

    writer.appendLine("def extrapolate():");
    writer.appendLine("  diff = [er_cmd_servo_q[0] - er_cmd_servo_q_last[0], er_cmd_servo_q[1] - er_cmd_servo_q_last[1], er_cmd_servo_q[2] - er_cmd_servo_q_last[2], er_cmd_servo_q[3] - er_cmd_servo_q_last[3], er_cmd_servo_q[4] - er_cmd_servo_q_last[4], er_cmd_servo_q[5] - er_cmd_servo_q_last[5]]");
    writer.appendLine("  er_cmd_servo_q_last = er_cmd_servo_q");
    writer.appendLine("  er_cmd_servo_q = [er_cmd_servo_q[0] + diff[0], er_cmd_servo_q[1] + diff[1], er_cmd_servo_q[2] + diff[2], er_cmd_servo_q[3] + diff[3], er_cmd_servo_q[4] + diff[4], er_cmd_servo_q[5] + diff[5]]");

    writer.appendLine("  return er_cmd_servo_q");
    writer.appendLine("end");

    writer.appendLine("thread servoThread():");
    writer.appendLine("  state = SERVO_IDLE");
    writer.appendLine("  while er_control_mode == MODE_SERVOJ:");
    writer.appendLine("    enter_critical");
    writer.appendLine("    q = er_cmd_servo_q");
    writer.appendLine("    do_extrapolate = False");
    writer.appendLine("    if (er_cmd_servo_state == SERVO_IDLE):");
    writer.appendLine("      do_extrapolate = True");
    writer.appendLine("    end");
    writer.appendLine("    state = er_cmd_servo_state");
    writer.appendLine("    if er_cmd_servo_state > SERVO_UNINITIALIZED:");
    writer.appendLine("      er_cmd_servo_state = SERVO_IDLE");
    writer.appendLine("    end");

    writer.appendLine("    if do_extrapolate:");
    writer.appendLine("      er_extrapolate_count = er_extrapolate_count + 1");
    writer.appendLine("      if er_extrapolate_count > er_extrapolate_max_count:");
    writer.appendLine("        er_extrapolate_max_count = er_extrapolate_count");
    writer.appendLine("      end");

    writer.appendLine("      q = extrapolate()");
    writer.appendLine("      servoj(q, t=steptime, lookahead_time=0.05, gain=100)");

    writer.appendLine("    elif state == SERVO_RUNNING:");
    writer.appendLine("      er_extrapolate_count = 0");
    writer.appendLine("      servoj(q, t=steptime, lookahead_time=0.05, gain=100)");
    writer.appendLine("    else:");
    writer.appendLine("      er_extrapolate_count = 0");
    writer.appendLine("      sync()");
    writer.appendLine("    end");
    writer.appendLine("    exit_critical");
    writer.appendLine("  end");
    writer.appendLine("  stopj(4.0)");
    writer.appendLine("end");

    writer.appendLine("# Helpers for speed control");
    writer.appendLine("def set_speed(qd):");
    writer.appendLine("  er_cmd_servo_qd = qd");
    writer.appendLine("  er_control_mode = MODE_SPEEDJ");
    writer.appendLine("end");

    writer.appendLine("thread speedThread():");
    writer.appendLine("  while er_control_mode == MODE_SPEEDJ:");
    writer.appendLine("    qd = er_cmd_servo_qd");
    writer.appendLine("    speedj(qd, 40.0, steptime)");
    writer.appendLine("  end");
    writer.appendLine("  stopj(5.0)");
    writer.appendLine("end");

    //NSChange  End
  }

  // IP helper functions
  public void setHostIP(String ip) {
    if ("".equals(ip)) {
      resetToDefaultIP();
    } else {
      model.set(HOST_IP, ip);
    }
  }

  public String getHostIP() {
    return model.get(HOST_IP, DEFAULT_IP);
  }

  private void resetToDefaultIP() {
    model.set(HOST_IP, DEFAULT_IP);
  }

  public KeyboardTextInput getInputForIPTextField() {
    KeyboardTextInput keyboInput = keyboardFactory.createIPAddressKeyboardInput();
    keyboInput.setInitialValue(getHostIP());
    return keyboInput;
  }

  public KeyboardInputCallback<String> getCallbackForIPTextField() {
    return new KeyboardInputCallback<String>() {
      @Override
      public void onOk(String value) {
        setHostIP(value);
        view.UpdateIPTextField(value);
      }
    };
  }

  // port helper functions
  public void setHostPort(Integer port) {
    if ("".equals(port)) {
      resetToDefaultPort();
    } else {
      model.set(PORT_NR, port);
    }
  }

  public Integer getCustomPort() {
    return model.get(PORT_NR, DEFAULT_PORT);
  }

  private void resetToDefaultPort() {
    model.set(PORT_NR, DEFAULT_PORT);
  }

  public KeyboardNumberInput<Integer> getInputForPortTextField() {
    KeyboardNumberInput<Integer> keyboInput = keyboardFactory.createIntegerKeypadInput();
    keyboInput.setInitialValue(getCustomPort());
    return keyboInput;
  }

  public KeyboardInputCallback<Integer> getCallbackForPortTextField() {
    return new KeyboardInputCallback<Integer>() {
      @Override
      public void onOk(Integer value) {
        setHostPort(value);
        view.UpdatePortTextField(value);
      }
    };
  }

  // name helper functions
  public void setName(String name) {
    if ("".equals(name)) {
      resetToDefaultName();
    } else {
      model.set(NAME, name);
    }
  }

  public String getName() {
    return model.get(NAME, DEFAULT_NAME);
  }

  private void resetToDefaultName() {
    model.set(NAME, DEFAULT_NAME);
  }

  public KeyboardTextInput getInputForNameTextField() {
    KeyboardTextInput keyboInput = keyboardFactory.createStringKeyboardInput();
    keyboInput.setInitialValue(getName());
    return keyboInput;
  }

  public KeyboardInputCallback<String> getCallbackForNameTextField() {
    return new KeyboardInputCallback<String>() {
      @Override
      public void onOk(String value) {
        setName(value);
        view.UpdateNameTextField(value);
      }
    };
  }

  public String getControlLoop(ScriptWriter writer) {
    if (programRequested == false) {
      // Request and split the program
      sender = new RequestProgram(getHostIP(), getCustomPort());
      sender.requestAndSplitProgram();

      // Append header to the ur program.
      writer.appendRaw(sender.getHeader());

      programRequested = true;
    }
    return sender.getControlLoop();
  }
}
