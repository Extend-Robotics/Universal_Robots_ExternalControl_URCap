//-- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//-- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Lea Steffen steffen@fzi.de
* \date    2019-04-18
*
*/
//----------------------------------------------------------------------

package com.fzi.externalcontrol.impl;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.domain.ProgramAPI;
import com.ur.urcap.api.domain.data.DataModel;
import com.ur.urcap.api.domain.undoredo.UndoRedoManager;
import com.ur.urcap.api.domain.undoredo.UndoableChanges;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputCallback;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputFactory;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;

public class ExternalControlProgramNodeContribution implements ProgramNodeContribution {
  private static final String ADVANCED_PARAM_KEY = "showadvancedparam";
  private static final boolean ADVANCED_PARAM_DEFAULT = false;
  private static final String MAX_LOST_PACKAGES = "maxlostpackages";
  private static final String MAX_LOST_PACKAGES_DEFAULT_VALUE = "1000";
  private static final String GAIN_SERVO_J = "gain_servo_j";
  private static final String GAIN_SERVO_J_DEFAULT_VALUE = "0";
  private final ProgramAPI programAPI;
  private final DataModel model;
  private final ExternalControlProgramNodeView view;
  private final KeyboardInputFactory keyboardFactory;
  private final UndoRedoManager undoRedoManager;
  private String control_loop;     //NSChange Added new Variable to hold the string 
  private static String SERVER_IP_REPLACE = "{{SERVER_IP_REPLACE}}";
  private static final String REVERSE_SOCKET = "reverse_socket";
  private static String SERVER_PORT_REPLACE = "{{SERVER_PORT_REPLACE}}";

  public ExternalControlProgramNodeContribution(
      ProgramAPIProvider apiProvider, ExternalControlProgramNodeView view, DataModel model) {
    this.programAPI = apiProvider.getProgramAPI();
    this.undoRedoManager = apiProvider.getProgramAPI().getUndoRedoManager();
    this.keyboardFactory =
        apiProvider.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory();
    this.model = model;
    this.view = view;
  }

  @Override
  public void openView() {
    view.updateInfoLabel(getInstallation().getHostIP(), getInstallation().getCustomPort());
  }

  @Override
  public void closeView() {}

  @Override
  public String getTitle() {
    return "Control by " + getInstallation().getName();
  }

  @Override
  public boolean isDefined() {
    return true;
  }
  //Add the Program method here
  @Override
  public void generateScript(ScriptWriter writer) {
  // NSChanges Start
    SERVER_IP_REPLACE = getInstallation().getHostIP();
    SERVER_PORT_REPLACE = "50001";
    writer.appendLine("socket_open(\"" + SERVER_IP_REPLACE + "\", " + SERVER_PORT_REPLACE +  ", \""+ REVERSE_SOCKET + "\")");
    writer.appendLine("er_control_mode = MODE_UNINITIALIZED");
    writer.appendLine("thread_move = 0");
    writer.appendLine("global er_keepalive = -2");
    writer.appendLine("params_mult = socket_read_binary_integer(1+6+1, \""+ REVERSE_SOCKET + "\", 0)");
    writer.appendLine("er_keepalive = params_mult[1]");
    writer.appendLine("while er_keepalive > 0 and er_control_mode > MODE_STOPPED:");
    writer.appendLine("  enter_critical");
    writer.appendLine("  params_mult = socket_read_binary_integer(1+6+1, \""+ REVERSE_SOCKET +"\", 0.02) # steptime could work as well, but does not work in simulation");
    writer.appendLine("  if params_mult[0] > 0:");
    writer.appendLine("    er_keepalive = params_mult[1]");
    writer.appendLine("    if er_control_mode != params_mult[8]:");
    writer.appendLine("      er_control_mode = params_mult[8]");
    writer.appendLine("      join thread_move");
    writer.appendLine("      if er_control_mode == MODE_SERVOJ:");
    writer.appendLine("        thread_move = run servoThread()");
    writer.appendLine("      elif er_control_mode == MODE_SPEEDJ:");
    writer.appendLine("        thread_move = run speedThread()");
    writer.appendLine("      end");
    writer.appendLine("    end");
    writer.appendLine("    if er_control_mode == MODE_SERVOJ:");
    writer.appendLine("      q = [params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate, params_mult[7] / MULT_jointstate]");
    writer.appendLine("      set_servo_setpoint(q)");
    writer.appendLine("    elif er_control_mode == MODE_SPEEDJ:");
    writer.appendLine("      qd = [params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate, params_mult[7] / MULT_jointstate]");
    writer.appendLine("      set_speed(qd)");
    writer.appendLine("    end");
    writer.appendLine("  else:");
    writer.appendLine("    er_keepalive = er_keepalive - 1");
    writer.appendLine("  end");
    writer.appendLine("  exit_critical");
    writer.appendLine("end");
    writer.appendLine("er_control_mode = MODE_STOPPED");
    writer.appendLine("join thread_move");
    writer.appendLine("socket_close(\""+ REVERSE_SOCKET +"\")");
    // String controlLoop = getInstallation().getControlLoop(writer);
    // writer.appendRaw(controlLoop);
    // NSChanges END
  }

  private ExternalControlInstallationNodeContribution getInstallation() {
    return programAPI.getInstallationNode(ExternalControlInstallationNodeContribution.class);
  }

  public void setParam(final String key, final String value, final String default_val) {
    undoRedoManager.recordChanges(new UndoableChanges() {
      @Override
      public void executeChanges() {
        if ("".equals(value)) {
          resetToDefaultValue(key, default_val);
        } else {
          model.set(key, value);
        }
      }
    });
  }

  public String getParam(String key, String default_val) {
    return model.get(key, default_val);
  }

  private void resetToDefaultValue(String key, String default_val) {
    model.set(key, default_val);
  }

  private boolean getAdvancedParam() {
    return model.get(ADVANCED_PARAM_KEY, ADVANCED_PARAM_DEFAULT);
  }

  public void setAdvancedParam(final boolean show) {
    updateAdvancedParam(show);
    // UndoRedoManager is necessary in program node but not in installation node
    // (see here:
    // https://plus.universal-robots.com/apidoc/40237/com/ur/urcap/api/domain/undoredo/undoredomanager.html)
    undoRedoManager.recordChanges(new UndoableChanges() {
      @Override
      public void executeChanges() {
        model.set(ADVANCED_PARAM_KEY, show);
      }
    });
  }

  private void updateAdvancedParam(boolean enable) {
    if (enable) {
      view.showAdvancedParameters(true);
    } else {
      view.showAdvancedParameters(false);
    }
  }

  public KeyboardTextInput getInputForMaxLostPackages() {
    KeyboardTextInput keyboardInput = keyboardFactory.createStringKeyboardInput();
    keyboardInput.setInitialValue(getParam(MAX_LOST_PACKAGES, MAX_LOST_PACKAGES_DEFAULT_VALUE));
    return keyboardInput;
  }

  public KeyboardInputCallback<String> getCallbackForMaxLostPackages() {
    return new KeyboardInputCallback<String>() {
      @Override
      public void onOk(String value) {
        setParam(MAX_LOST_PACKAGES, value, MAX_LOST_PACKAGES_DEFAULT_VALUE);
        view.updateMaxLostPackages_TF(value);
      }
    };
  }

  public KeyboardTextInput getInputForGainServoj() {
    KeyboardTextInput keyboardInput = keyboardFactory.createStringKeyboardInput();
    keyboardInput.setInitialValue(getParam(GAIN_SERVO_J, GAIN_SERVO_J_DEFAULT_VALUE));
    return keyboardInput;
  }

  public KeyboardInputCallback<String> getCallbackForGainServoj() {
    return new KeyboardInputCallback<String>() {
      @Override
      public void onOk(String value) {
        setParam(GAIN_SERVO_J, value, GAIN_SERVO_J_DEFAULT_VALUE);
        view.updateGainServoj_TF(value);
      }
    };
  }
}
