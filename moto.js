/**
 * Moto Vehicle Module for Dream Light Engine
 * Uses existing Physics API functions from main engine
 *
 * Required global functions (from index.html):
 * - createDynamicBody(world, x, y, options)
 * - addCircleFixture(body, radius, options)
 * - addPolygonFixture(body, vertices, options)
 * - createRevoluteJoint(world, bodyA, bodyB, anchorA, anchorB, options)
 * - getVec2(x, y)
 * - Physics (global Box2D-WASM reference)
 */

(function(global) {
  'use strict';

  // Moto configuration
  const MOTO = {
    // Chassis (main body frame)
    chassis: {
      vertices: [
        { x: 0.4, y: -0.3 },
        { x: 0.5, y: 0.4 },
        { x: -0.75, y: 0.16 },
        { x: -0.35, y: -0.3 }
      ],
      density: 1.5,
      friction: 1.0,
      restitution: 0.2
    },

    // Wheels
    frontWheel: { x: 0.65, y: -0.35, radius: 0.35, density: 1.8, friction: 1.4, restitution: 0.2 },
    rearWheel: { x: -0.65, y: -0.35, radius: 0.35, density: 1.8, friction: 1.4, restitution: 0.2 },

    // Rider body parts
    head: { x: -0.1, y: 1.1, radius: 0.15, density: 0.4 },
    torso: {
      x: -0.15, y: 0.65,
      vertices: [
        { x: 0.1, y: -0.35 },
        { x: 0.12, y: 0.2 },
        { x: -0.15, y: 0.25 },
        { x: -0.12, y: -0.35 }
      ],
      density: 0.5
    },
    upperLeg: {
      x: 0.1, y: 0.25,
      vertices: [
        { x: 0.25, y: -0.08 },
        { x: 0.25, y: 0.06 },
        { x: -0.25, y: 0.1 },
        { x: -0.25, y: -0.06 }
      ],
      density: 0.4
    },
    lowerLeg: {
      x: 0.35, y: 0.0,
      vertices: [
        { x: 0.18, y: -0.06 },
        { x: 0.18, y: 0.06 },
        { x: -0.18, y: 0.08 },
        { x: -0.18, y: -0.06 }
      ],
      density: 0.4
    },
    upperArm: {
      x: 0.05, y: 0.75,
      vertices: [
        { x: 0.06, y: -0.18 },
        { x: 0.06, y: 0.15 },
        { x: -0.06, y: 0.18 },
        { x: -0.06, y: -0.18 }
      ],
      density: 0.3
    },
    lowerArm: {
      x: 0.25, y: 0.55,
      vertices: [
        { x: 0.18, y: -0.05 },
        { x: 0.18, y: 0.04 },
        { x: -0.18, y: 0.05 },
        { x: -0.18, y: -0.04 }
      ],
      density: 0.3
    },

    // Physics
    maxSpeed: 50,
    acceleration: 12,
    leanForce: 8,
    jumpForce: 15
  };

  /**
   * Moto class - Motorcycle with rider
   */
  class Moto {
    constructor() {
      // Bodies
      this.chassis = null;
      this.frontWheel = null;
      this.rearWheel = null;

      // Rider bodies
      this.head = null;
      this.torso = null;
      this.upperLeg = null;
      this.lowerLeg = null;
      this.upperArm = null;
      this.lowerArm = null;

      // Joints
      this.joints = [];
      this.riderJoints = []; // Joints connecting rider to bike

      // State
      this.alive = true;
      this.direction = 1; // 1 = right, -1 = left
      this.world = null;
      this.spawnX = 0;
      this.spawnY = 0;
    }

    /**
     * Spawn moto at position
     */
    spawn(world, x, y, dir = 1) {
      this.world = world;
      this.spawnX = x;
      this.spawnY = y;
      this.direction = dir;
      this.alive = true;

      // Create chassis
      this.chassis = createDynamicBody(world, x, y, { angularDamping: 0.5 });
      const chassisVerts = MOTO.chassis.vertices.map(v => ({ x: v.x * dir, y: v.y }));
      addPolygonFixture(this.chassis, chassisVerts, {
        density: MOTO.chassis.density,
        friction: MOTO.chassis.friction,
        restitution: MOTO.chassis.restitution
      });

      // Create wheels
      this.rearWheel = createDynamicBody(world, x + MOTO.rearWheel.x * dir, y + MOTO.rearWheel.y, {});
      addCircleFixture(this.rearWheel, MOTO.rearWheel.radius, {
        density: MOTO.rearWheel.density,
        friction: MOTO.rearWheel.friction,
        restitution: MOTO.rearWheel.restitution
      });

      this.frontWheel = createDynamicBody(world, x + MOTO.frontWheel.x * dir, y + MOTO.frontWheel.y, {});
      addCircleFixture(this.frontWheel, MOTO.frontWheel.radius, {
        density: MOTO.frontWheel.density,
        friction: MOTO.frontWheel.friction,
        restitution: MOTO.frontWheel.restitution
      });

      // Connect wheels to chassis with revolute joints
      const rearAnchor = getVec2(MOTO.rearWheel.x * dir, MOTO.rearWheel.y);
      const frontAnchor = getVec2(MOTO.frontWheel.x * dir, MOTO.frontWheel.y);
      const wheelCenter = getVec2(0, 0);

      this.joints.push(createRevoluteJoint(world, this.chassis, this.rearWheel, rearAnchor, wheelCenter, {}));
      this.joints.push(createRevoluteJoint(world, this.chassis, this.frontWheel, frontAnchor, wheelCenter, {}));

      // Create rider
      this._createRider(world, x, y, dir);
    }

    /**
     * Create rider body parts and joints
     */
    _createRider(world, x, y, dir) {
      const cfg = MOTO;

      // Head (circle)
      this.head = createDynamicBody(world, x + cfg.head.x * dir, y + cfg.head.y, {});
      addCircleFixture(this.head, cfg.head.radius, { density: cfg.head.density, friction: 0.5, restitution: 0.2 });

      // Torso
      this.torso = createDynamicBody(world, x + cfg.torso.x * dir, y + cfg.torso.y, {});
      const torsoVerts = cfg.torso.vertices.map(v => ({ x: v.x * dir, y: v.y }));
      addPolygonFixture(this.torso, torsoVerts, { density: cfg.torso.density, friction: 0.5, restitution: 0.1 });

      // Upper leg
      this.upperLeg = createDynamicBody(world, x + cfg.upperLeg.x * dir, y + cfg.upperLeg.y, {});
      const upperLegVerts = cfg.upperLeg.vertices.map(v => ({ x: v.x * dir, y: v.y }));
      addPolygonFixture(this.upperLeg, upperLegVerts, { density: cfg.upperLeg.density, friction: 0.5, restitution: 0.1 });

      // Lower leg
      this.lowerLeg = createDynamicBody(world, x + cfg.lowerLeg.x * dir, y + cfg.lowerLeg.y, {});
      const lowerLegVerts = cfg.lowerLeg.vertices.map(v => ({ x: v.x * dir, y: v.y }));
      addPolygonFixture(this.lowerLeg, lowerLegVerts, { density: cfg.lowerLeg.density, friction: 0.5, restitution: 0.1 });

      // Upper arm
      this.upperArm = createDynamicBody(world, x + cfg.upperArm.x * dir, y + cfg.upperArm.y, {});
      const upperArmVerts = cfg.upperArm.vertices.map(v => ({ x: v.x * dir, y: v.y }));
      addPolygonFixture(this.upperArm, upperArmVerts, { density: cfg.upperArm.density, friction: 0.5, restitution: 0.1 });

      // Lower arm
      this.lowerArm = createDynamicBody(world, x + cfg.lowerArm.x * dir, y + cfg.lowerArm.y, {});
      const lowerArmVerts = cfg.lowerArm.vertices.map(v => ({ x: v.x * dir, y: v.y }));
      addPolygonFixture(this.lowerArm, lowerArmVerts, { density: cfg.lowerArm.density, friction: 0.5, restitution: 0.1 });

      // === RIDER JOINTS ===

      // Neck (head to torso)
      const neckAnchorTorso = getVec2(0.0 * dir, 0.25);
      const neckAnchorHead = getVec2(0, -0.1);
      this.joints.push(createRevoluteJoint(world, this.torso, this.head, neckAnchorTorso, neckAnchorHead, {
        enableLimit: true, lowerAngle: -0.5, upperAngle: 0.5
      }));

      // Hip (torso to upper leg)
      const hipAnchorTorso = getVec2(-0.05 * dir, -0.35);
      const hipAnchorLeg = getVec2(-0.2 * dir, 0.05);
      this.joints.push(createRevoluteJoint(world, this.torso, this.upperLeg, hipAnchorTorso, hipAnchorLeg, {
        enableLimit: true, lowerAngle: -1.2, upperAngle: 0.5
      }));

      // Knee (upper leg to lower leg)
      const kneeAnchorUpper = getVec2(0.22 * dir, 0);
      const kneeAnchorLower = getVec2(-0.15 * dir, 0);
      this.joints.push(createRevoluteJoint(world, this.upperLeg, this.lowerLeg, kneeAnchorUpper, kneeAnchorLower, {
        enableLimit: true, lowerAngle: -2.0, upperAngle: 0.1
      }));

      // Shoulder (torso to upper arm)
      const shoulderAnchorTorso = getVec2(0.08 * dir, 0.15);
      const shoulderAnchorArm = getVec2(0, 0.15);
      this.joints.push(createRevoluteJoint(world, this.torso, this.upperArm, shoulderAnchorTorso, shoulderAnchorArm, {
        enableLimit: true, lowerAngle: -1.5, upperAngle: 1.5
      }));

      // Elbow (upper arm to lower arm)
      const elbowAnchorUpper = getVec2(0.0 * dir, -0.15);
      const elbowAnchorLower = getVec2(-0.15 * dir, 0);
      this.joints.push(createRevoluteJoint(world, this.upperArm, this.lowerArm, elbowAnchorUpper, elbowAnchorLower, {
        enableLimit: true, lowerAngle: -2.0, upperAngle: 0.1
      }));

      // === RIDER TO BIKE CONNECTIONS ===

      // Foot to chassis (ankle)
      const footAnchorLeg = getVec2(0.15 * dir, 0);
      const footAnchorChassis = getVec2(0.0 * dir, -0.1);
      const ankleJoint = createRevoluteJoint(world, this.lowerLeg, this.chassis, footAnchorLeg, footAnchorChassis, {});
      this.riderJoints.push(ankleJoint);
      this.joints.push(ankleJoint);

      // Hand to chassis (wrist/handlebar)
      const handAnchorArm = getVec2(0.15 * dir, 0);
      const handAnchorChassis = getVec2(0.35 * dir, 0.3);
      const wristJoint = createRevoluteJoint(world, this.lowerArm, this.chassis, handAnchorArm, handAnchorChassis, {});
      this.riderJoints.push(wristJoint);
      this.joints.push(wristJoint);
    }

    /**
     * Update moto physics based on input
     */
    update(input) {
      if (!this.alive || !this.chassis) return;

      const dir = this.direction;

      // Accelerate (W or Up)
      if (input.up) {
        const vel = this.rearWheel.GetAngularVelocity();
        if (Math.abs(vel) < MOTO.maxSpeed) {
          this.rearWheel.ApplyTorque(MOTO.acceleration * dir, true);
          this.frontWheel.ApplyTorque(MOTO.acceleration * dir * 0.3, true);
        }
      }

      // Brake (S or Down)
      if (input.down) {
        const rearVel = this.rearWheel.GetAngularVelocity();
        const frontVel = this.frontWheel.GetAngularVelocity();
        this.rearWheel.ApplyTorque(-rearVel * 0.5, true);
        this.frontWheel.ApplyTorque(-frontVel * 0.5, true);
      }

      // Lean back (A or Left) - wheelie
      if (input.left) {
        const force = new Physics.b2Vec2(0, -MOTO.leanForce);
        const point = this.chassis.GetWorldPoint(getVec2(-0.5 * dir, 0));
        this.chassis.ApplyForce(force, point, true);
      }

      // Lean forward (D or Right) - stoppie
      if (input.right) {
        const force = new Physics.b2Vec2(0, -MOTO.leanForce);
        const point = this.chassis.GetWorldPoint(getVec2(0.5 * dir, 0));
        this.chassis.ApplyForce(force, point, true);
      }
    }

    /**
     * Get moto position (chassis center)
     */
    getPosition() {
      if (!this.chassis) return { x: this.spawnX, y: this.spawnY };
      const pos = this.chassis.GetPosition();
      return { x: pos.get_x(), y: pos.get_y() };
    }

    /**
     * Flip direction
     */
    flip() {
      this.direction *= -1;
    }

    /**
     * Eject rider from bike
     */
    eject() {
      if (!this.alive) return;

      // Destroy rider-to-bike joints
      for (const joint of this.riderJoints) {
        try {
          this.world.DestroyJoint(joint);
        } catch (e) {}
      }
      this.riderJoints = [];

      // Apply ejection force to torso
      if (this.torso) {
        const force = new Physics.b2Vec2(MOTO.leanForce * this.direction, -MOTO.jumpForce);
        this.torso.ApplyForceToCenter(force, true);
      }

      this.alive = false;
    }

    /**
     * Destroy all bodies and joints
     */
    destroy() {
      // Destroy all joints first
      for (const joint of this.joints) {
        try {
          this.world.DestroyJoint(joint);
        } catch (e) {}
      }
      this.joints = [];
      this.riderJoints = [];

      // Destroy bodies
      const bodies = [
        this.chassis, this.frontWheel, this.rearWheel,
        this.head, this.torso, this.upperLeg, this.lowerLeg, this.upperArm, this.lowerArm
      ];
      for (const body of bodies) {
        if (body) {
          try {
            this.world.DestroyBody(body);
          } catch (e) {}
        }
      }

      this.chassis = null;
      this.frontWheel = null;
      this.rearWheel = null;
      this.head = null;
      this.torso = null;
      this.upperLeg = null;
      this.lowerLeg = null;
      this.upperArm = null;
      this.lowerArm = null;
      this.alive = false;
    }

    /**
     * Get all bodies for rendering
     */
    getBodies() {
      return [
        { body: this.chassis, type: 'chassis', color: '#ff6600' },
        { body: this.frontWheel, type: 'wheel', color: '#333' },
        { body: this.rearWheel, type: 'wheel', color: '#333' },
        { body: this.head, type: 'head', color: '#ffcc99' },
        { body: this.torso, type: 'torso', color: '#3366ff' },
        { body: this.upperLeg, type: 'limb', color: '#3366ff' },
        { body: this.lowerLeg, type: 'limb', color: '#3366ff' },
        { body: this.upperArm, type: 'limb', color: '#ffcc99' },
        { body: this.lowerArm, type: 'limb', color: '#ffcc99' }
      ].filter(b => b.body !== null);
    }
  }

  // Export
  global.Moto = Moto;
  global.MOTO_CONFIG = MOTO;

})(typeof window !== 'undefined' ? window : this);
