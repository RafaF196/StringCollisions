package main;

import java.io.File;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.lwjgl.input.Keyboard;
import org.lwjgl.opengl.Display;
import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;

import collisions.CollisionMaster;
import entities.Camera;
import entities.Entity;
import fontMeshCreator.FontType;
import fontMeshCreator.GUIText;
import fontRender.TextMaster;
import models.RawModel;
import models.TexturedModel;
import objFileLoader.ObjFileLoader;
import particles.ParticleSystem;
import render.DisplayManager;
import render.Loader;
import render.MasterRenderer;
import textures.ModelTexture;

public class MainTest {

	public static void main(String[] args) {

		DisplayManager.createDisplay();
		
		Loader loader = new Loader();
		MasterRenderer renderer = new MasterRenderer(loader);
		TextMaster.init(loader);
		
		//Camera camera = new Camera(new Vector3f(0, 10, 6), 20.0f, 0.0f);
		Camera camera = new Camera(new Vector3f(10, 6, 0), 20.0f, -90.0f);
		List<Entity> entities = new ArrayList<Entity>();
		
		FontType font = new FontType(loader.loadTexture("candara"), new File("res/candara.fnt"));
		GUIText fric_text = new GUIText("Friction coef.: 0.05", 1.6f, font, new Vector2f(0.008f, 0.018f), 1f, false);
		fric_text.setColour(1, 0, 0);
		GUIText boun_text = new GUIText("Bouncing coef.: 0.90", 1.6f, font, new Vector2f(0.008f, 0.068f), 1f, false);
		boun_text.setColour(1, 0, 0);
		NumberFormat nf = NumberFormat.getNumberInstance(Locale.ENGLISH);
		DecimalFormat decimalFormat = (DecimalFormat)nf;

		float delta, friction = 0.05f, bouncing = 0.90f;
		String numberAsString;
		
		Boolean FrictionChangePressed = false, BouncingChangePressed = false;
		
		TexturedModel particle = loadModel("particle", "red", loader);
		
		ParticleSystem ps = new ParticleSystem(particle, new Vector3f(0,6,-1), new Vector3f(0.0f,0.0f,0.2f), 80);
		
		CollisionMaster cm = new CollisionMaster();
		RawModel mod = cm.addSphere(loader, new Vector3f(0, 3, -1), 3f);
		entities.add(new Entity(loadModel(mod, "cyan", loader), new Vector3f(0, 3, -1), 0, 3f));
		
		RawModel mod2 = cm.addPlane(loader, new Vector3f(0, 1, 0), 15f);
		entities.add(new Entity(loadModel(mod2, "green", loader), new Vector3f(0, 0, 0), 0, 1f));
		
		while(!Display.isCloseRequested()) {
			
			//delta = DisplayManager.getFrameTimeSeconds();
			delta = 0.01f;
	        
		    // Modify the Friction value
			if (Keyboard.isKeyDown(Keyboard.KEY_LEFT)) {
				if (friction > 0.01f && !FrictionChangePressed) {
					friction -= 0.05;
					if (friction < 0f) friction = 0;
					
					fric_text.remove();
					numberAsString = decimalFormat.format(friction);
					fric_text = new GUIText("Friction coef.: " + numberAsString, 1.6f, font,
							new Vector2f(0.008f, 0.018f), 1f, false);
					fric_text.setColour(1, 0, 0);
					
					FrictionChangePressed = true;
				}
			} else if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT)) {
				if (friction < 0.99f && !FrictionChangePressed) {
					friction += 0.05f;
					
					fric_text.remove();
					numberAsString = decimalFormat.format(friction);
					fric_text = new GUIText("Friction coef.: " + numberAsString, 1.6f, font,
							new Vector2f(0.008f, 0.018f), 1f, false);
					fric_text.setColour(1, 0, 0);
					
					FrictionChangePressed = true;
				}
			} else {
				FrictionChangePressed = false;
			}
			
			// Modify the Bouncing value
			if (Keyboard.isKeyDown(Keyboard.KEY_DOWN)) {
				if (bouncing > 0.01f && !BouncingChangePressed) {
					bouncing -= 0.05;
					if (bouncing < 0f) bouncing = 0;
					
					boun_text.remove();
					numberAsString = decimalFormat.format(bouncing);
					boun_text = new GUIText("Bouncing coef.: " + numberAsString, 1.6f, font,
							new Vector2f(0.008f, 0.068f), 1f, false);
					boun_text.setColour(1, 0, 0);
					
					BouncingChangePressed = true;
				}
			} else if (Keyboard.isKeyDown(Keyboard.KEY_UP)) {
				if (bouncing < 0.99f && !BouncingChangePressed) {
					bouncing += 0.05f;
					
					boun_text.remove();
					numberAsString = decimalFormat.format(bouncing);
					boun_text = new GUIText("Bouncing coef.: " + numberAsString, 1.6f, font,
							new Vector2f(0.008f, 0.068f), 1f, false);
					boun_text.setColour(1, 0, 0);
					
					BouncingChangePressed = true;
				}
			} else {
				BouncingChangePressed = false;
			}
			
			camera.move(delta);
			renderer.renderScene(entities, ps, camera, 1);
			for (int i = 0; i < 5; i++) {
				if (delta < 1) ps.update(delta/5, cm, friction, bouncing);
			}
			TextMaster.render();
			DisplayManager.updateDisplay();
			
		}
		
		TextMaster.cleanUp();
		renderer.cleanUp();
		loader.cleanUp();
		DisplayManager.closeDisplay();

	}
	
	private static TexturedModel loadModel(String modelName, String textureName, Loader loader){
		RawModel model = ObjFileLoader.loadOBJ(modelName, loader);
		ModelTexture texture = new ModelTexture(loader.loadTexture(textureName));
		return new TexturedModel(model, texture);
	}
	
	private static TexturedModel loadModel(RawModel model, String textureName, Loader loader){
		ModelTexture texture = new ModelTexture(loader.loadTexture(textureName));
		return new TexturedModel(model, texture);
	}

}
