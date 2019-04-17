package com.fzi.generateScript.impl;

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

public class GenerateScriptProgramNodeContribution implements ProgramNodeContribution{
	private static final String ADVANCED_PARAM_KEY = "showadvancedparam";
	private static final boolean ADVANCED_PARAM_DEFAULT = false;
	private static final String MAX_LOST_PACKAGES= "maxlostpackages";
	private static final String MAX_LOST_PACKAGES_DEFAULT_VALUE= "1000";
	private static final String GAIN_SERVO_J= "gain_servo_j";
	private static final String GAIN_SERVO_J_DEFAULT_VALUE= "0";
	private final ProgramAPI programAPI;
	private final DataModel model;
	private final GenerateScriptProgramNodeView view;
	private final KeyboardInputFactory keyboardFactory;
	private final UndoRedoManager undoRedoManager;
	

	
	public GenerateScriptProgramNodeContribution(ProgramAPIProvider apiProvider, GenerateScriptProgramNodeView view,
			DataModel model) {
		this.programAPI = apiProvider.getProgramAPI();
		this.undoRedoManager = apiProvider.getProgramAPI().getUndoRedoManager();
		this.keyboardFactory = apiProvider.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory();
		this.model = model;
		this.view = view;
	}
	

	@Override
	public void openView() {
	}

	@Override
	public void closeView() {
	}

	@Override
	public String getTitle() {
		return "GenerateScript";
	}

	@Override
	public boolean isDefined() {
		return true;
	}
	
	/*
	@Override
	public void generateScript(ScriptWriter writer) {
		writer.appendRaw("popup(\"" + getInstallation().getHostIP() + "\" )");
	}*/
	
	@Override
	public void generateScript(ScriptWriter writer) {
		//writer.appendRaw("popup(\"" + getParam(MAX_LOST_PACKAGES, MAX_LOST_PACKAGES_DEFAULT_VALUE) + "\" )");
		writer.appendRaw("popup(\"" + getParam(GAIN_SERVO_J, GAIN_SERVO_J_DEFAULT_VALUE) + "\" )");
	}
	
	/*
	@Override
	public void generateScript(ScriptWriter writer) {
		writer.appendRaw("popup(\"IP: \"  \"" + getInstallation().getHostIP() + "\"   \"      MLP:  \"  \"" + getInstallation().getHostIP() + "\"  )");
	}*/
	
	
	private GenerateScriptInstallationNodeContribution getInstallation() {
		return programAPI.getInstallationNode(GenerateScriptInstallationNodeContribution.class);
	}
	
	
	public void setParam(final String key, final String value , final String default_val) {
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
		undoRedoManager.recordChanges(new UndoableChanges() {
			@Override
			public void executeChanges() {
				model.set(ADVANCED_PARAM_KEY, show);
			}
		});
		
	}
	
	
	
	
	private void updateAdvancedParam(boolean enable) {
		if(enable) {
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
