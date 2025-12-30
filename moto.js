/**
 * Moto Vehicle Module for Dream Light Engine
 * Box2D-WASM 7.0.0 (Emscripten) compatible implementation
 * Based on Liquid.lab patterns but adapted for WASM API
 *
 * Requires: Box2D-WASM globals (b2Vec2, b2BodyDef, b2PolygonShape, b2CircleShape, etc.)
 *           via window.ModulePhysics or global b2* classes
 */

(function(global) {
  'use strict';

  // ═══════════════════════════════════════════════════════════════════════
  // BOX2D-WASM 7.0.0 HELPER METHODS
  // These use the Emscripten set_* API, not the old property-based API
  // ═══════════════════════════════════════════════════════════════════════

  function extendWorld(worldInstance) {
    if (worldInstance._motoExtended) return;
    worldInstance._motoExtended = true;

    // Get Box2D module reference
    var b2 = global.ModulePhysics || global;

    /**
     * Create a body with fixture
     */
    worldInstance.add = function(obj) {
      obj = obj || {};

      // Create BodyDef
      var bd = new b2BodyDef();
      var bodyType = obj.type || 'dynamic';

      switch (bodyType) {
        case 'dynamic': bd.set_type(b2_dynamicBody); break;
        case 'static': bd.set_type(b2_staticBody); break;
        case 'kinematic': bd.set_type(b2_kinematicBody); break;
        default: bd.set_type(b2_dynamicBody);
      }

      // Set position
      var pos = new b2Vec2(obj.x || 0, obj.y || 0);
      bd.set_position(pos);
      bd.set_angle(obj.angle || 0);
      bd.set_allowSleep(obj.allowSleep !== undefined ? obj.allowSleep : true);
      bd.set_awake(obj.awake !== undefined ? obj.awake : true);
      bd.set_bullet(obj.bullet || false);
      bd.set_fixedRotation(obj.fixedRotation || false);

      // Create body
      var body = this.CreateBody(bd);

      // Create and attach fixture
      var fd = new b2FixtureDef();
      fd.set_density(obj.density || 0);
      fd.set_friction(obj.friction || 0.2);
      fd.set_restitution(obj.restitution || 0.1);
      fd.set_isSensor(obj.isSensor || false);

      // Set filter if needed
      if (obj.groupIndex !== undefined) {
        var filter = fd.get_filter();
        filter.set_groupIndex(obj.groupIndex);
        fd.set_filter(filter);
      }

      // Create shape based on type
      var shape = this.createShape(obj);
      fd.set_shape(shape);

      // Create fixture
      body.CreateFixture(fd);

      // Set velocities if provided
      if (obj.angularVelocity) body.SetAngularVelocity(obj.angularVelocity);
      if (obj.linearVelocity) {
        var lv = new b2Vec2(obj.linearVelocity.x, obj.linearVelocity.y);
        body.SetLinearVelocity(lv);
        if (b2.destroy) b2.destroy(lv);
      }

      // Cleanup WASM memory
      if (b2.destroy) {
        b2.destroy(pos);
        b2.destroy(shape);
        // Don't destroy fd - it's copied by CreateFixture
      }

      return body;
    };

    /**
     * Create shape based on obj properties
     */
    worldInstance.createShape = function(obj) {
      var b2 = global.ModulePhysics || global;
      var shapeName = obj.shape || 'box';
      var shape;

      switch (shapeName) {
        case 'polygon':
          shape = new b2PolygonShape();
          var len = obj.vertices.length;
          var dx = obj.mx || 0;
          var dy = obj.my || 0;
          var vecs = [];

          // Create b2Vec2 array for vertices (pairs: x, y, x, y...)
          for (var i = 0; i < len; i += 2) {
            vecs.push(new b2Vec2(obj.vertices[i] + dx, obj.vertices[i + 1] + dy));
          }

          // Set polygon vertices
          shape.Set(vecs, vecs.length);

          // Cleanup vertex vectors
          if (b2.destroy) {
            for (var j = 0; j < vecs.length; j++) {
              b2.destroy(vecs[j]);
            }
          }
          break;

        case 'box':
          shape = new b2PolygonShape();
          shape.SetAsBox((obj.w || 1) * 0.5, (obj.h || 1) * 0.5);
          break;

        case 'circle':
          shape = new b2CircleShape();
          shape.set_m_radius(obj.radius || 1);
          break;

        case 'edge':
          shape = new b2EdgeShape();
          shape.Set(obj.p1, obj.p2);
          break;

        default:
          shape = new b2PolygonShape();
          shape.SetAsBox(0.5, 0.5);
      }

      return shape;
    };

    /**
     * Create joint
     */
    worldInstance.addJoint = function(obj) {
      obj = obj || {};
      var b2 = global.ModulePhysics || global;
      var type = obj.type || 'revolute';
      var jd;
      var joint = null;

      var bodyA = obj.bodyA;
      var bodyB = obj.bodyB;

      switch (type) {
        case 'revolute':
          jd = new b2RevoluteJointDef();

          // Use set_* API for Emscripten bindings
          jd.set_bodyA(bodyA);
          jd.set_bodyB(bodyB);

          // Convert world anchor to local anchors
          var anchorWorld = obj.axis || bodyB.GetWorldCenter();
          var localAnchorA = bodyA.GetLocalPoint(anchorWorld);
          var localAnchorB = bodyB.GetLocalPoint(anchorWorld);

          jd.set_localAnchorA(localAnchorA);
          jd.set_localAnchorB(localAnchorB);

          jd.set_enableMotor(obj.enableMotor || false);
          jd.set_motorSpeed(obj.motorSpeed || 0);
          jd.set_maxMotorTorque(obj.maxMotorTorque || 0);
          jd.set_enableLimit(obj.enableLimit || false);
          jd.set_lowerAngle(obj.lowerAngle || 0);
          jd.set_upperAngle(obj.upperAngle || 0);
          jd.set_collideConnected(obj.collideConnected || false);

          joint = this.CreateJoint(jd);
          break;

        case 'prismatic':
          jd = new b2PrismaticJointDef();

          jd.set_bodyA(bodyA);
          jd.set_bodyB(bodyB);

          // Convert world anchor to local anchors
          var pAnchorWorld = obj.axis || bodyB.GetWorldCenter();
          var pLocalAnchorA = bodyA.GetLocalPoint(pAnchorWorld);
          var pLocalAnchorB = bodyB.GetLocalPoint(pAnchorWorld);

          jd.set_localAnchorA(pLocalAnchorA);
          jd.set_localAnchorB(pLocalAnchorB);

          // Set axis direction (normalized)
          var axisDir = obj.angle || new b2Vec2(0, 1);
          var localAxis = bodyA.GetLocalVector(axisDir);
          jd.set_localAxisA(localAxis);

          jd.set_enableMotor(obj.enableMotor || false);
          jd.set_motorSpeed(obj.motorSpeed || 0);
          jd.set_maxMotorForce(obj.maxMotorForce || 0);
          jd.set_enableLimit(obj.enableLimit || false);
          jd.set_lowerTranslation(obj.lowerTranslation || 0);
          jd.set_upperTranslation(obj.upperTranslation || 0);
          jd.set_collideConnected(obj.collideConnected || false);

          joint = this.CreateJoint(jd);
          break;

        case 'distance':
          jd = new b2DistanceJointDef();
          jd.set_bodyA(bodyA);
          jd.set_bodyB(bodyB);
          if (obj.length !== undefined) jd.set_length(obj.length);
          if (obj.frequencyHz !== undefined) jd.set_frequencyHz(obj.frequencyHz);
          if (obj.dampingRatio !== undefined) jd.set_dampingRatio(obj.dampingRatio);
          jd.set_collideConnected(obj.collideConnected || false);
          joint = this.CreateJoint(jd);
          break;
      }

      return joint;
    };
  }

  // ═══════════════════════════════════════════════════════════════════════
  // HELPER FUNCTIONS
  // ═══════════════════════════════════════════════════════════════════════

  function cloneCST(constant, mirror, pos) {
    var obj = {};
    for (var e in constant) obj[e] = constant[e];

    if (obj.shape === 'polygon' && mirror === -1) {
      obj.vertices = reversVertices(constant.vertices);
    }
    if (obj.angle && mirror === -1) {
      obj.angle = mirror * constant.angle;
    }
    if (pos) {
      obj.x = pos.x + mirror * constant.x;
      obj.y = pos.y + constant.y;
    }
    return obj;
  }

  function reversVertices(vertices) {
    var newVertices = [];
    var len = vertices.length;
    for (var i = 0; i < len; i += 2) {
      newVertices.unshift(vertices[i + 1]);
      newVertices.unshift(-vertices[i]);
    }
    return newVertices;
  }

  function rotatePoint(point, angle, rotationAxe) {
    return new b2Vec2(
      rotationAxe.x + point.x * Math.cos(angle) - point.y * Math.sin(angle),
      rotationAxe.y + point.x * Math.sin(angle) + point.y * Math.cos(angle)
    );
  }

  // ═══════════════════════════════════════════════════════════════════════
  // CONSTANTS (exact values from Liquid.lab)
  // ═══════════════════════════════════════════════════════════════════════

  var Constants = {
    max_moto_speed: 70.0,
    moto_acceleration: 9.0,
    biker_force: 11.0,
    air_density: 0.03,

    // Main body (chassis)
    body: {
      shape: 'polygon', x: 0.0, y: 1.0, angle: 0,
      density: 1.5, restitution: 0.5, friction: 1.0,
      vertices: [0.4, -0.3, 0.50, 0.40, -0.75, 0.16, -0.35, -0.3],
      groupIndex: -1, allowSleep: false
    },

    // Wheels
    left_wheel: {
      shape: 'circle', x: -0.70, y: 0.48, angle: 0,
      radius: 0.35, density: 1.8, restitution: 0.3, friction: 1.4,
      groupIndex: -1, allowSleep: false
    },
    right_wheel: {
      shape: 'circle', x: 0.70, y: 0.48, angle: 0,
      radius: 0.35, density: 1.8, restitution: 0.3, friction: 1.4,
      groupIndex: -1, allowSleep: false
    },

    // Axles (suspension arms)
    left_axle: {
      shape: 'polygon', x: 0.0, y: 1.0, angle: 0,
      density: 1.0, restitution: 0.5, friction: 1.0,
      vertices: [-0.10, -0.30, -0.25, -0.30, -0.80, -0.58, -0.65, -0.58],
      groupIndex: -1, allowSleep: false
    },
    right_axle: {
      shape: 'polygon', x: 0.0, y: 1.0, angle: 0,
      density: 1.5, restitution: 0.5, friction: 1.0,
      vertices: [0.58, -0.02, 0.48, -0.02, 0.66, -0.58, 0.76, -0.58],
      groupIndex: -1, allowSleep: false
    },

    // Rider parts
    head: {
      shape: 'circle', x: -0.27, y: 2.26, radius: 0.18, angle: 0,
      density: 0.4, restitution: 0.0, friction: 1.0,
      groupIndex: -1, allowSleep: false
    },
    torso: {
      shape: 'polygon', x: -0.31, y: 1.89, angle: -Math.PI / 30.0,
      density: 0.4, restitution: 0.0, friction: 1.0,
      vertices: [0.10, -0.55, 0.13, 0.15, -0.20, 0.22, -0.18, -0.55],
      groupIndex: -1, allowSleep: false
    },
    lower_leg: {
      shape: 'polygon', x: 0.07, y: 0.90, angle: -Math.PI / 6.0,
      density: 0.4, restitution: 0.0, friction: 1.0,
      vertices: [0.2, -0.33, 0.2, -0.27, 0.00, -0.2, 0.02, 0.33, -0.17, 0.33, -0.14, -0.33],
      groupIndex: -1, allowSleep: false
    },
    upper_leg: {
      shape: 'polygon', x: -0.15, y: 1.27, angle: -Math.PI / 11.0,
      density: 0.4, restitution: 0.0, friction: 1.0,
      vertices: [0.4, -0.14, 0.4, 0.07, -0.4, 0.14, -0.4, -0.08],
      groupIndex: -1, allowSleep: false
    },
    lower_arm: {
      shape: 'polygon', x: 0.07, y: 1.54, angle: -Math.PI / 10.0,
      density: 0.4, restitution: 0.0, friction: 1.0,
      vertices: [0.28, -0.07, 0.28, 0.04, -0.30, 0.07, -0.30, -0.06],
      groupIndex: -1, allowSleep: false
    },
    upper_arm: {
      shape: 'polygon', x: -0.20, y: 1.85, angle: Math.PI / 10.0,
      density: 0.4, restitution: 0.0, friction: 1.0,
      vertices: [0.09, -0.29, 0.09, 0.22, -0.11, 0.26, -0.10, -0.29],
      groupIndex: -1, allowSleep: false
    },

    // Suspension settings
    left_suspension: {
      angle: { x: 0, y: 1 },
      lowerTranslation: -0.03,
      upperTranslation: 0.20,
      back_force: 3.00,
      rigidity: 8.00
    },
    right_suspension: {
      angle: { x: -0.2, y: 1 },
      lowerTranslation: -0.01,
      upperTranslation: 0.20,
      back_force: 3.00,
      rigidity: 4.00
    },

    // Joint anchors (relative to body part)
    ankle: { axe_position: { x: -0.18, y: -0.2 } },
    wrist: { axe_position: { x: 0.25, y: -0.07 } },
    knee: { axe_position: { x: 0.12, y: 0.28 } },
    elbow: { axe_position: { x: 0.03, y: -0.21 } },
    shoulder: { axe_position: { x: -0.12, y: 0.22 } },
    hip: { axe_position: { x: -0.25, y: 0.14 } }
  };

  // ═══════════════════════════════════════════════════════════════════════
  // RIDER CLASS
  // ═══════════════════════════════════════════════════════════════════════

  function Rider(world, moto) {
    this.bodyList = ['head', 'torso', 'lower_leg', 'upper_leg', 'lower_arm', 'upper_arm'];
    this.world = world;
    this.moto = moto;
    this.mirror = moto.mirror;
    this.isDead = false;

    // Body references
    this.head = null;
    this.torso = null;
    this.lower_leg = null;
    this.upper_leg = null;
    this.lower_arm = null;
    this.upper_arm = null;

    // Joint references
    this.neck_joint = null;
    this.ankle_joint = null;
    this.wrist_joint = null;
    this.knee_joint = null;
    this.elbow_joint = null;
    this.shoulder_joint = null;
    this.hip_joint = null;
  }

  Rider.prototype = {
    constructor: Rider,

    init: function() {
      this.mirror = this.moto.mirror;
      var pos = this.moto.startPos;

      // Create all rider body parts
      for (var i = 0; i < this.bodyList.length; i++) {
        var name = this.bodyList[i];
        this[name] = this.world.add(cloneCST(Constants[name], this.mirror, pos));
      }

      // Create joints
      this.neck_joint = this.createNeckJoint();
      this.ankle_joint = this.createJoint(Constants.ankle, this.lower_leg, this.moto.body);
      this.wrist_joint = this.createJoint(Constants.wrist, this.lower_arm, this.moto.body);
      this.knee_joint = this.createJoint(Constants.knee, this.lower_leg, this.upper_leg);
      this.elbow_joint = this.createJoint(Constants.elbow, this.upper_arm, this.lower_arm);
      this.shoulder_joint = this.createJoint(Constants.shoulder, this.upper_arm, this.torso);
      this.hip_joint = this.createJoint(Constants.hip, this.upper_leg, this.torso);

      this.isDead = false;
    },

    createNeckJoint: function() {
      var anchor = this.head.GetWorldCenter();
      return this.world.addJoint({
        type: 'revolute',
        bodyA: this.head,
        bodyB: this.torso,
        axis: anchor
      });
    },

    createJoint: function(cst, part1, part2) {
      var pos = part1.GetWorldCenter();
      var axis = new b2Vec2(
        pos.get_x() + this.mirror * cst.axe_position.x,
        pos.get_y() + cst.axe_position.y
      );

      return this.world.addJoint({
        type: 'revolute',
        bodyA: part1,
        bodyB: part2,
        axis: axis
      });
    },

    killJoint: function() {
      if (!this.isDead) {
        try { this.world.DestroyJoint(this.ankle_joint); } catch (e) {}
        try { this.world.DestroyJoint(this.wrist_joint); } catch (e) {}
        this.isDead = true;
      }
    },

    eject: function() {
      if (!this.moto.isDead) {
        this.moto.kill();
        var force = new b2Vec2(150.0 * this.mirror, 0);
        var angle = this.mirror * this.moto.body.GetAngle() + Math.PI / 4.0;
        var adjusted = rotatePoint(force, angle, { x: 0, y: 0 });
        this.torso.ApplyForce(adjusted, this.torso.GetWorldCenter(), true);
      }
    },

    destroy: function() {
      if (!this.isDead) {
        try { this.world.DestroyJoint(this.ankle_joint); } catch (e) {}
        try { this.world.DestroyJoint(this.wrist_joint); } catch (e) {}
      }
      try { this.world.DestroyJoint(this.neck_joint); } catch (e) {}
      try { this.world.DestroyJoint(this.knee_joint); } catch (e) {}
      try { this.world.DestroyJoint(this.elbow_joint); } catch (e) {}
      try { this.world.DestroyJoint(this.shoulder_joint); } catch (e) {}
      try { this.world.DestroyJoint(this.hip_joint); } catch (e) {}

      for (var i = 0; i < this.bodyList.length; i++) {
        var name = this.bodyList[i];
        if (this[name]) {
          try { this.world.DestroyBody(this[name]); } catch (e) {}
          this[name] = null;
        }
      }
    }
  };

  // ═══════════════════════════════════════════════════════════════════════
  // MOTO CLASS
  // ═══════════════════════════════════════════════════════════════════════

  function Moto(world, mirror) {
    this.bodyList = ['body', 'left_wheel', 'right_wheel', 'left_axle', 'right_axle'];

    this.world = world;

    // Extend world with helper methods
    extendWorld(this.world);

    this.mirror = mirror ? -1 : 1;
    this.rider = new Rider(world, this);
    this.isDead = false;
    this.isDestroy = true;

    this.startPos = { x: 0, y: 0 };
    this.position = { x: 0, y: 0 };

    // Body references
    this.body = null;
    this.left_wheel = null;
    this.right_wheel = null;
    this.left_axle = null;
    this.right_axle = null;

    // Joint references
    this.left_revolute_joint = null;
    this.right_revolute_joint = null;
    this.left_prismatic_joint = null;
    this.right_prismatic_joint = null;
  }

  Moto.prototype = {
    constructor: Moto,

    spawn: function(x, y) {
      this.startPos = { x: x, y: y };
      this.init();
    },

    init: function() {
      var pos = this.startPos;

      // Create all moto bodies
      for (var i = 0; i < this.bodyList.length; i++) {
        var name = this.bodyList[i];
        this[name] = this.world.add(cloneCST(Constants[name], this.mirror, pos));
      }

      // Wheel-axle revolute joints
      this.left_revolute_joint = this.createRevoluteJoint(this.left_axle, this.left_wheel);
      this.right_revolute_joint = this.createRevoluteJoint(this.right_axle, this.right_wheel);

      // Body-axle prismatic joints (suspension)
      this.left_prismatic_joint = this.createPrismaticJoint(this.left_axle, Constants.left_suspension);
      this.right_prismatic_joint = this.createPrismaticJoint(this.right_axle, Constants.right_suspension);

      // Init rider
      this.rider.mirror = this.mirror;
      this.rider.init();

      this.isDestroy = false;
      this.isDead = false;
    },

    createRevoluteJoint: function(axle, wheel) {
      return this.world.addJoint({
        type: 'revolute',
        bodyA: axle,
        bodyB: wheel,
        axis: wheel.GetWorldCenter()
      });
    },

    createPrismaticJoint: function(axle, cst) {
      var axisVec = new b2Vec2(this.mirror * cst.angle.x, cst.angle.y);
      return this.world.addJoint({
        type: 'prismatic',
        bodyA: this.body,
        bodyB: axle,
        axis: axle.GetWorldCenter(),
        angle: axisVec,
        lowerTranslation: cst.lowerTranslation,
        upperTranslation: cst.upperTranslation,
        enableLimit: true,
        enableMotor: true
      });
    },

    update: function(input) {
      if (this.isDead || this.isDestroy) return;

      // Acceleration
      if (input.up) {
        this.left_wheel.ApplyTorque(-this.mirror * Constants.moto_acceleration, true);
      }

      // Brake
      if (input.down) {
        this.right_wheel.SetAngularVelocity(0);
        this.left_wheel.SetAngularVelocity(0);
      }

      // Lean (wheelie/stoppie)
      var biker_force = Constants.biker_force;
      if ((input.left && this.mirror === 1) || (input.right && this.mirror === -1)) {
        this.wheeling(biker_force);
      }
      if ((input.right && this.mirror === 1) || (input.left && this.mirror === -1)) {
        this.wheeling(-biker_force * 0.8);
      }

      // Natural wheel drag
      if (!input.up && !input.down) {
        var v = this.left_wheel.GetAngularVelocity();
        this.left_wheel.ApplyTorque(Math.abs(v) >= 0.2 ? -v * 0.1 : 0, true);
        v = this.right_wheel.GetAngularVelocity();
        this.right_wheel.ApplyTorque(Math.abs(v) >= 0.2 ? -v * 0.01 : 0, true);
      }

      // Update suspension
      this.updateSuspension();

      // Speed limits
      this.limitSpeed(this.left_wheel);
      this.limitSpeed(this.right_wheel);

      // Air drag
      var vel = this.body.GetLinearVelocity();
      var vx = vel.get_x();
      var squared_speed = vx * vx;
      var drag_force = Constants.air_density * squared_speed * 0.025;
      this.body.SetLinearDamping(drag_force);

      // Update position
      this.position = this.getPosition();
    },

    limitSpeed: function(wheel) {
      var vel = wheel.GetAngularVelocity();
      if (vel > Constants.max_moto_speed) {
        wheel.SetAngularVelocity(Constants.max_moto_speed);
      } else if (vel < -Constants.max_moto_speed) {
        wheel.SetAngularVelocity(-Constants.max_moto_speed);
      }
    },

    updateSuspension: function() {
      // Left suspension
      var cst = Constants.left_suspension;
      var trans = this.left_prismatic_joint.GetJointTranslation();
      var max = cst.rigidity + Math.abs(cst.rigidity * 100 * Math.pow(trans, 2));
      var speed = -cst.back_force * trans;
      this.left_prismatic_joint.SetMaxMotorForce(max);
      this.left_prismatic_joint.SetMotorSpeed(speed);

      // Right suspension
      cst = Constants.right_suspension;
      trans = this.right_prismatic_joint.GetJointTranslation();
      max = cst.rigidity + Math.abs(cst.rigidity * 100 * Math.pow(trans, 2));
      speed = -cst.back_force * trans;
      this.right_prismatic_joint.SetMaxMotorForce(max);
      this.right_prismatic_joint.SetMotorSpeed(speed);
    },

    wheeling: function(force) {
      var moto_angle = this.mirror * this.body.GetAngle();
      this.body.ApplyTorque(this.mirror * force * 0.50, true);

      // Apply force to rider torso and legs for more realistic physics
      var cos_a = Math.cos(moto_angle);
      var sin_a = Math.sin(moto_angle);

      var force_torso = new b2Vec2(
        this.mirror * (-force) * cos_a,
        this.mirror * this.mirror * (-force) * sin_a
      );
      this.rider.torso.ApplyForce(force_torso, this.rider.torso.GetWorldCenter(), true);

      var force_leg = new b2Vec2(
        this.mirror * force * cos_a,
        this.mirror * this.mirror * force * sin_a
      );
      this.rider.lower_leg.ApplyForce(force_leg, this.rider.lower_leg.GetWorldCenter(), true);
    },

    getPosition: function() {
      var p = this.body.GetWorldCenter();
      return {
        x: p.get_x() - this.mirror * Constants.body.x,
        y: p.get_y() - Constants.body.y
      };
    },

    flip: function() {
      if (!this.isDead) {
        this.mirror *= -1;
      }
    },

    eject: function() {
      this.rider.eject();
    },

    kill: function() {
      if (!this.isDead) {
        this.isDead = true;
        this.rider.killJoint();
      }
    },

    destroy: function() {
      this.rider.destroy();

      try { this.world.DestroyJoint(this.left_revolute_joint); } catch (e) {}
      try { this.world.DestroyJoint(this.left_prismatic_joint); } catch (e) {}
      try { this.world.DestroyJoint(this.right_revolute_joint); } catch (e) {}
      try { this.world.DestroyJoint(this.right_prismatic_joint); } catch (e) {}

      for (var i = 0; i < this.bodyList.length; i++) {
        var name = this.bodyList[i];
        if (this[name]) {
          try { this.world.DestroyBody(this[name]); } catch (e) {}
          this[name] = null;
        }
      }

      this.isDestroy = true;
    }
  };

  // Export
  global.Moto = Moto;
  global.Rider = Rider;
  global.MotoConstants = Constants;

})(typeof window !== 'undefined' ? window : this);
