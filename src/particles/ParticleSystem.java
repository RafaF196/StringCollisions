package particles;

import java.util.ArrayList;
import java.util.Random;

import org.lwjgl.util.vector.Vector3f;

import collisions.CollisionMaster;
import models.TexturedModel;

public class ParticleSystem {
	
	private TexturedModel model;
	private Vector3f initialPos;
	private Float numParticles;
	private Float L;
	
	ArrayList<Particle> particles = new ArrayList<Particle>();
	
	Random rand = new Random();

	public ParticleSystem(TexturedModel model, Vector3f initialPos, Float numParticles) {
		this.model = model;
		this.initialPos = initialPos;
		this.numParticles = numParticles;
	}
	
	public ParticleSystem(TexturedModel model, Vector3f initialPos, Vector3f offset, Integer numParticles) {
		this.model = model;
		this.initialPos = initialPos;
		this.L = offset.length();
		addString(initialPos, offset, numParticles);
	}

	public TexturedModel getModel() {
		return model;
	}
	
	public Vector3f getPosition() {
		return initialPos;
	}
	
	public Float getNumParticles() {
		return numParticles;
	}
	
	public Float getL() {
		return L;
	}
	
	public ArrayList<Vector3f> getOffsets() {
		ArrayList<Vector3f> os = new ArrayList<Vector3f>();
		for (int i = 0; i < particles.size(); i++) os.add(particles.get(i).getOffset());
		return os;
	}
	
	public void addParticle(Float delta) {
		Vector3f initV = new Vector3f((rand.nextFloat()-0.5f)*4, 0.01f, (rand.nextFloat()-0.5f)*4);
		Particle p = new Particle(initialPos, initV, 0.001f, delta, 9.8f, 10);
		particles.add(p);
	}
	
	public void addString(Vector3f initPos, Vector3f offset, Integer numParticles) {
		Vector3f totalOffset = new Vector3f(0,0,0);
		for (int i = 0; i < numParticles; i++) {
			Vector3f initV = new Vector3f(0f, 0f, 0f);
			Particle p = new Particle(initialPos, totalOffset, initV, 0.001f, 0.01f, 9.8f, 1000, i == 0);
			particles.add(p);
			Vector3f.add(totalOffset, offset, totalOffset);
		}
	}
	
	public void update(Float delta, CollisionMaster cm, Float friction, Float bouncing) {
		Boolean delete;
		for (int i = 0; i < particles.size(); i++) {
			delete = particles.get(i).update(particles, this.L, i, delta, cm, friction, bouncing);
			if (delete) {
				particles.remove(i);
				i--;
			}
		}
	}
	
}
