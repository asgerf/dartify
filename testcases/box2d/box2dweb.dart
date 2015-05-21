library box2dweb;

import 'dart:js';

abstract class b2TimeStep {
  factory b2TimeStep() = _b2TimeStep;

  Set(step);
  static b2TimeStep() => _b2TimeStep.b2TimeStep();
}

abstract class b2Transform {
  factory b2Transform(pos, r) = _b2Transform;

  b2Transform(pos, r);
  Initialize(pos, r);
  SetIdentity();
  Set(x);
  GetAngle();
  static b2Transform() => _b2Transform.b2Transform();
}

abstract class b2Sweep {
  factory b2Sweep() = _b2Sweep;

  Set(other);
  Copy();
  GetTransform(xf, alpha);
  Advance(t);
  static b2Sweep() => _b2Sweep.b2Sweep();
}

abstract class b2Color {
  factory b2Color(rr, gg, bb) = _b2Color;

  b2Color(rr, gg, bb);
  Set(rr, gg, bb);
  static b2Color() => _b2Color.b2Color();
}

abstract class b2Controller {
  factory b2Controller() = _b2Controller;

  Step(step);
  Draw(debugDraw);
  AddBody(body);
  RemoveBody(body);
  Clear();
  GetNext();
  GetWorld();
  GetBodyList();
  static b2Controller() => _b2Controller.b2Controller();
}

abstract class b2Vec2 {
  factory b2Vec2(x_, y_) = _b2Vec2;

  b2Vec2(x_, y_);
  SetZero();
  Set(x_, y_);
  SetV(v);
  GetNegative();
  NegativeSelf();
  Copy();
  Add(v);
  Subtract(v);
  Multiply(a);
  MulM(A);
  MulTM(A);
  CrossVF(s);
  CrossFV(s);
  MinV(b);
  MaxV(b);
  Abs();
  Length();
  LengthSquared();
  Normalize();
  IsValid();
  static b2Vec2() => _b2Vec2.b2Vec2();
  static Make(x_, y_) => _b2Vec2.Make(x_, y_);
}

abstract class b2Contact {
  factory b2Contact() = _b2Contact;

  GetManifold();
  GetWorldManifold(worldManifold);
  IsTouching();
  IsContinuous();
  SetSensor(sensor);
  IsSensor();
  SetEnabled(flag);
  IsEnabled();
  GetNext();
  GetFixtureA();
  GetFixtureB();
  FlagForFiltering();
  b2Contact();
  Reset(fixtureA, fixtureB);
  Update(listener);
  Evaluate();
  ComputeTOI(sweepA, sweepB);
  static b2Contact() => _b2Contact.b2Contact();
}

abstract class b2WorldManifold {
  factory b2WorldManifold() = _b2WorldManifold;

  b2WorldManifold();
  Initialize(manifold, xfA, radiusA, xfB, radiusB);
  static b2WorldManifold() => _b2WorldManifold.b2WorldManifold();
}

abstract class b2PositionSolverManifold {
  factory b2PositionSolverManifold() = _b2PositionSolverManifold;

  b2PositionSolverManifold();
  Initialize(cc);
  static b2PositionSolverManifold() => _b2PositionSolverManifold.b2PositionSolverManifold();
}

abstract class b2TOIInput {
  factory b2TOIInput() = _b2TOIInput;

  static b2TOIInput() => _b2TOIInput.b2TOIInput();
}

abstract class b2DistanceProxy {
  factory b2DistanceProxy() = _b2DistanceProxy;

  Set(shape);
  GetSupport(d);
  GetSupportVertex(d);
  GetVertexCount();
  GetVertex(index);
  static b2DistanceProxy() => _b2DistanceProxy.b2DistanceProxy();
}

abstract class b2Shape {
  factory b2Shape() = _b2Shape;

  Copy();
  Set(other);
  GetType();
  TestPoint(xf, p);
  RayCast(output, input, transform);
  ComputeAABB(aabb, xf);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  b2Shape();
  static b2Shape() => _b2Shape.b2Shape();
  static TestOverlap(shape1, transform1, shape2, transform2) => _b2Shape.TestOverlap(shape1, transform1, shape2, transform2);
}

abstract class b2Mat22 {
  factory b2Mat22() = _b2Mat22;

  b2Mat22();
  Set(angle);
  SetVV(c1, c2);
  Copy();
  SetM(m);
  AddM(m);
  SetIdentity();
  SetZero();
  GetAngle();
  GetInverse(out);
  Solve(out, bX, bY);
  Abs();
  static b2Mat22() => _b2Mat22.b2Mat22();
  static FromAngle(angle) => _b2Mat22.FromAngle(angle);
  static FromVV(c1, c2) => _b2Mat22.FromVV(c1, c2);
}

abstract class b2SimplexCache {
  factory b2SimplexCache() = _b2SimplexCache;

  static b2SimplexCache() => _b2SimplexCache.b2SimplexCache();
}

abstract class b2DistanceInput {
  factory b2DistanceInput() = _b2DistanceInput;

  static b2DistanceInput() => _b2DistanceInput.b2DistanceInput();
}

abstract class b2SeparationFunction {
  factory b2SeparationFunction() = _b2SeparationFunction;

  Initialize(cache, proxyA, transformA, proxyB, transformB);
  Evaluate(transformA, transformB);
  static b2SeparationFunction() => _b2SeparationFunction.b2SeparationFunction();
}

abstract class b2DistanceOutput {
  factory b2DistanceOutput() = _b2DistanceOutput;

  static b2DistanceOutput() => _b2DistanceOutput.b2DistanceOutput();
}

abstract class b2Simplex {
  factory b2Simplex() = _b2Simplex;

  b2Simplex();
  ReadCache(cache, proxyA, transformA, proxyB, transformB);
  WriteCache(cache);
  GetSearchDirection();
  GetClosestPoint();
  GetWitnessPoints(pA, pB);
  GetMetric();
  Solve2();
  Solve3();
  static b2Simplex() => _b2Simplex.b2Simplex();
}

abstract class b2SimplexVertex {
  factory b2SimplexVertex() = _b2SimplexVertex;

  Set(other);
  static b2SimplexVertex() => _b2SimplexVertex.b2SimplexVertex();
}

abstract class ClipVertex {
  factory ClipVertex() = _ClipVertex;

  Set(other);
  static ClipVertex() => _ClipVertex.ClipVertex();
}

abstract class b2ContactID {
  factory b2ContactID() = _b2ContactID;

  b2ContactID();
  Set(id);
  Copy();
  static b2ContactID() => _b2ContactID.b2ContactID();
}

abstract class Features {
  factory Features() = _Features;

  static Features() => _Features.Features();
}

abstract class b2ContactImpulse {
  factory b2ContactImpulse() = _b2ContactImpulse;

  static b2ContactImpulse() => _b2ContactImpulse.b2ContactImpulse();
}

abstract class b2ContactPoint {
  factory b2ContactPoint() = _b2ContactPoint;

  static b2ContactPoint() => _b2ContactPoint.b2ContactPoint();
}

abstract class b2ContactListener {
  factory b2ContactListener() = _b2ContactListener;

  BeginContact(contact);
  EndContact(contact);
  PreSolve(contact, oldManifold);
  PostSolve(contact, impulse);
  static b2ContactListener() => _b2ContactListener.b2ContactListener();
}

abstract class b2ContactFilter {
  factory b2ContactFilter() = _b2ContactFilter;

  ShouldCollide(fixtureA, fixtureB);
  RayCollide(userData, fixture);
  static b2ContactFilter() => _b2ContactFilter.b2ContactFilter();
}

abstract class b2JointDef {
  factory b2JointDef() = _b2JointDef;

  b2JointDef();
  static b2JointDef() => _b2JointDef.b2JointDef();
}

abstract class b2Joint {
  factory b2Joint(def) = _b2Joint;

  GetType();
  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetBodyA();
  GetBodyB();
  GetNext();
  GetUserData();
  SetUserData(data);
  IsActive();
  b2Joint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  FinalizeVelocityConstraints();
  SolvePositionConstraints(baumgarte);
  static b2Joint() => _b2Joint.b2Joint();
  static Create(def, allocator) => _b2Joint.Create(def, allocator);
  static Destroy(joint, allocator) => _b2Joint.Destroy(joint, allocator);
}

abstract class b2Mat33 {
  factory b2Mat33(c1, c2, c3) = _b2Mat33;

  b2Mat33(c1, c2, c3);
  SetVVV(c1, c2, c3);
  Copy();
  SetM(m);
  AddM(m);
  SetIdentity();
  SetZero();
  Solve22(out, bX, bY);
  Solve33(out, bX, bY, bZ);
  static b2Mat33() => _b2Mat33.b2Mat33();
}

abstract class b2Vec3 {
  factory b2Vec3(x, y, z) = _b2Vec3;

  b2Vec3(x, y, z);
  SetZero();
  Set(x, y, z);
  SetV(v);
  GetNegative();
  NegativeSelf();
  Copy();
  Add(v);
  Subtract(v);
  Multiply(a);
  static b2Vec3() => _b2Vec3.b2Vec3();
}

abstract class b2DistanceJoint extends b2Joint {
  factory b2DistanceJoint(def) = _b2DistanceJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetLength();
  SetLength(length);
  GetFrequency();
  SetFrequency(hz);
  GetDampingRatio();
  SetDampingRatio(ratio);
  b2DistanceJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2DistanceJoint() => _b2DistanceJoint.b2DistanceJoint();
}

abstract class b2DistanceJointDef extends b2JointDef {
  factory b2DistanceJointDef() = _b2DistanceJointDef;

  b2DistanceJointDef();
  Initialize(bA, bB, anchorA, anchorB);
  static b2DistanceJointDef() => _b2DistanceJointDef.b2DistanceJointDef();
}

abstract class b2FrictionJoint extends b2Joint {
  factory b2FrictionJoint(def) = _b2FrictionJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  SetMaxForce(force);
  GetMaxForce();
  SetMaxTorque(torque);
  GetMaxTorque();
  b2FrictionJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2FrictionJoint() => _b2FrictionJoint.b2FrictionJoint();
}

abstract class b2FrictionJointDef extends b2JointDef {
  factory b2FrictionJointDef() = _b2FrictionJointDef;

  b2FrictionJointDef();
  Initialize(bA, bB, anchor);
  static b2FrictionJointDef() => _b2FrictionJointDef.b2FrictionJointDef();
}

abstract class b2GearJoint extends b2Joint {
  factory b2GearJoint(def) = _b2GearJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetRatio();
  SetRatio(ratio);
  b2GearJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2GearJoint() => _b2GearJoint.b2GearJoint();
}

abstract class b2GearJointDef extends b2JointDef {
  factory b2GearJointDef() = _b2GearJointDef;

  b2GearJointDef();
  static b2GearJointDef() => _b2GearJointDef.b2GearJointDef();
}

abstract class b2Jacobian {
  factory b2Jacobian() = _b2Jacobian;

  SetZero();
  Set(x1, a1, x2, a2);
  Compute(x1, a1, x2, a2);
  static b2Jacobian() => _b2Jacobian.b2Jacobian();
}

abstract class b2LineJoint extends b2Joint {
  factory b2LineJoint(def) = _b2LineJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetJointTranslation();
  GetJointSpeed();
  IsLimitEnabled();
  EnableLimit(flag);
  GetLowerLimit();
  GetUpperLimit();
  SetLimits(lower, upper);
  IsMotorEnabled();
  EnableMotor(flag);
  SetMotorSpeed(speed);
  GetMotorSpeed();
  SetMaxMotorForce(force);
  GetMaxMotorForce();
  GetMotorForce();
  b2LineJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2LineJoint() => _b2LineJoint.b2LineJoint();
}

abstract class b2LineJointDef extends b2JointDef {
  factory b2LineJointDef() = _b2LineJointDef;

  b2LineJointDef();
  Initialize(bA, bB, anchor, axis);
  static b2LineJointDef() => _b2LineJointDef.b2LineJointDef();
}

abstract class b2MouseJoint extends b2Joint {
  factory b2MouseJoint(def) = _b2MouseJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetTarget();
  SetTarget(target);
  GetMaxForce();
  SetMaxForce(maxForce);
  GetFrequency();
  SetFrequency(hz);
  GetDampingRatio();
  SetDampingRatio(ratio);
  b2MouseJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2MouseJoint() => _b2MouseJoint.b2MouseJoint();
}

abstract class b2MouseJointDef extends b2JointDef {
  factory b2MouseJointDef() = _b2MouseJointDef;

  b2MouseJointDef();
  static b2MouseJointDef() => _b2MouseJointDef.b2MouseJointDef();
}

abstract class b2PrismaticJoint extends b2Joint {
  factory b2PrismaticJoint(def) = _b2PrismaticJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetJointTranslation();
  GetJointSpeed();
  IsLimitEnabled();
  EnableLimit(flag);
  GetLowerLimit();
  GetUpperLimit();
  SetLimits(lower, upper);
  IsMotorEnabled();
  EnableMotor(flag);
  SetMotorSpeed(speed);
  GetMotorSpeed();
  SetMaxMotorForce(force);
  GetMotorForce();
  b2PrismaticJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2PrismaticJoint() => _b2PrismaticJoint.b2PrismaticJoint();
}

abstract class b2PrismaticJointDef extends b2JointDef {
  factory b2PrismaticJointDef() = _b2PrismaticJointDef;

  b2PrismaticJointDef();
  Initialize(bA, bB, anchor, axis);
  static b2PrismaticJointDef() => _b2PrismaticJointDef.b2PrismaticJointDef();
}

abstract class b2PulleyJoint extends b2Joint {
  factory b2PulleyJoint(def) = _b2PulleyJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetGroundAnchorA();
  GetGroundAnchorB();
  GetLength1();
  GetLength2();
  GetRatio();
  b2PulleyJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2PulleyJoint() => _b2PulleyJoint.b2PulleyJoint();
}

abstract class b2PulleyJointDef extends b2JointDef {
  factory b2PulleyJointDef() = _b2PulleyJointDef;

  b2PulleyJointDef();
  Initialize(bA, bB, gaA, gaB, anchorA, anchorB, r);
  static b2PulleyJointDef() => _b2PulleyJointDef.b2PulleyJointDef();
}

abstract class b2RevoluteJoint extends b2Joint {
  factory b2RevoluteJoint(def) = _b2RevoluteJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetJointAngle();
  GetJointSpeed();
  IsLimitEnabled();
  EnableLimit(flag);
  GetLowerLimit();
  GetUpperLimit();
  SetLimits(lower, upper);
  IsMotorEnabled();
  EnableMotor(flag);
  SetMotorSpeed(speed);
  GetMotorSpeed();
  SetMaxMotorTorque(torque);
  GetMotorTorque();
  b2RevoluteJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2RevoluteJoint() => _b2RevoluteJoint.b2RevoluteJoint();
}

abstract class b2RevoluteJointDef extends b2JointDef {
  factory b2RevoluteJointDef() = _b2RevoluteJointDef;

  b2RevoluteJointDef();
  Initialize(bA, bB, anchor);
  static b2RevoluteJointDef() => _b2RevoluteJointDef.b2RevoluteJointDef();
}

abstract class b2WeldJoint extends b2Joint {
  factory b2WeldJoint(def) = _b2WeldJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  b2WeldJoint(def);
  InitVelocityConstraints(step);
  SolveVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  static b2WeldJoint() => _b2WeldJoint.b2WeldJoint();
}

abstract class b2WeldJointDef extends b2JointDef {
  factory b2WeldJointDef() = _b2WeldJointDef;

  b2WeldJointDef();
  Initialize(bA, bB, anchor);
  static b2WeldJointDef() => _b2WeldJointDef.b2WeldJointDef();
}

abstract class b2Body {
  factory b2Body(bd, world) = _b2Body;

  connectEdges(s1, s2, angle1);
  CreateFixture(def);
  CreateFixture2(shape, density);
  DestroyFixture(fixture);
  SetPositionAndAngle(position, angle);
  SetTransform(xf);
  GetTransform();
  GetPosition();
  SetPosition(position);
  GetAngle();
  SetAngle(angle);
  GetWorldCenter();
  GetLocalCenter();
  SetLinearVelocity(v);
  GetLinearVelocity();
  SetAngularVelocity(omega);
  GetAngularVelocity();
  GetDefinition();
  ApplyForce(force, point);
  ApplyTorque(torque);
  ApplyImpulse(impulse, point);
  Split(callback);
  Merge(other);
  GetMass();
  GetInertia();
  GetMassData(data);
  SetMassData(massData);
  ResetMassData();
  GetWorldPoint(localPoint);
  GetWorldVector(localVector);
  GetLocalPoint(worldPoint);
  GetLocalVector(worldVector);
  GetLinearVelocityFromWorldPoint(worldPoint);
  GetLinearVelocityFromLocalPoint(localPoint);
  GetLinearDamping();
  SetLinearDamping(linearDamping);
  GetAngularDamping();
  SetAngularDamping(angularDamping);
  SetType(type);
  GetType();
  SetBullet(flag);
  IsBullet();
  SetSleepingAllowed(flag);
  SetAwake(flag);
  IsAwake();
  SetFixedRotation(fixed);
  IsFixedRotation();
  SetActive(flag);
  IsActive();
  IsSleepingAllowed();
  GetFixtureList();
  GetJointList();
  GetControllerList();
  GetContactList();
  GetNext();
  GetUserData();
  SetUserData(data);
  GetWorld();
  b2Body(bd, world);
  SynchronizeFixtures();
  SynchronizeTransform();
  ShouldCollide(other);
  Advance(t);
  static b2Body() => _b2Body.b2Body();
}

abstract class b2BodyDef {
  factory b2BodyDef() = _b2BodyDef;

  b2BodyDef();
  static b2BodyDef() => _b2BodyDef.b2BodyDef();
}

abstract class b2ContactManager {
  factory b2ContactManager() = _b2ContactManager;

  b2ContactManager();
  AddPair(proxyUserDataA, proxyUserDataB);
  FindNewContacts();
  Destroy(c);
  Collide();
  static b2ContactManager() => _b2ContactManager.b2ContactManager();
}

abstract class b2DebugDraw {
  factory b2DebugDraw() = _b2DebugDraw;

  b2DebugDraw();
  SetFlags(flags);
  GetFlags();
  AppendFlags(flags);
  ClearFlags(flags);
  SetSprite(sprite);
  GetSprite();
  SetDrawScale(drawScale);
  GetDrawScale();
  SetLineThickness(lineThickness);
  GetLineThickness();
  SetAlpha(alpha);
  GetAlpha();
  SetFillAlpha(alpha);
  GetFillAlpha();
  SetXFormScale(xformScale);
  GetXFormScale();
  DrawPolygon(vertices, vertexCount, color);
  DrawSolidPolygon(vertices, vertexCount, color);
  DrawCircle(center, radius, color);
  DrawSolidCircle(center, radius, axis, color);
  DrawSegment(p1, p2, color);
  DrawTransform(xf);
  static b2DebugDraw() => _b2DebugDraw.b2DebugDraw();
}

abstract class b2DestructionListener {
  factory b2DestructionListener() = _b2DestructionListener;

  SayGoodbyeJoint(joint);
  SayGoodbyeFixture(fixture);
  static b2DestructionListener() => _b2DestructionListener.b2DestructionListener();
}

abstract class b2FilterData {
  factory b2FilterData() = _b2FilterData;

  Copy();
  static b2FilterData() => _b2FilterData.b2FilterData();
}

abstract class b2Fixture {
  factory b2Fixture() = _b2Fixture;

  GetType();
  GetShape();
  SetSensor(sensor);
  IsSensor();
  SetFilterData(filter);
  GetFilterData();
  GetBody();
  GetNext();
  GetUserData();
  SetUserData(data);
  TestPoint(p);
  RayCast(output, input);
  GetMassData(massData);
  SetDensity(density);
  GetDensity();
  GetFriction();
  SetFriction(friction);
  GetRestitution();
  SetRestitution(restitution);
  GetAABB();
  b2Fixture();
  Create(body, xf, def);
  Destroy();
  CreateProxy(broadPhase, xf);
  DestroyProxy(broadPhase);
  Synchronize(broadPhase, transform1, transform2);
  static b2Fixture() => _b2Fixture.b2Fixture();
}

abstract class b2FixtureDef {
  factory b2FixtureDef() = _b2FixtureDef;

  b2FixtureDef();
  static b2FixtureDef() => _b2FixtureDef.b2FixtureDef();
}

abstract class b2Island {
  factory b2Island() = _b2Island;

  b2Island();
  Initialize(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver);
  Clear();
  Solve(step, gravity, allowSleep);
  SolveTOI(subStep);
  Report(constraints);
  AddBody(body);
  AddContact(contact);
  AddJoint(joint);
  static b2Island() => _b2Island.b2Island();
}

abstract class b2World {
  factory b2World(gravity, doSleep) = _b2World;

  b2World(gravity, doSleep);
  SetDestructionListener(listener);
  SetContactFilter(filter);
  SetContactListener(listener);
  SetDebugDraw(debugDraw);
  SetBroadPhase(broadPhase);
  Validate();
  GetProxyCount();
  CreateBody(def);
  DestroyBody(b);
  CreateJoint(def);
  DestroyJoint(j);
  AddController(c);
  RemoveController(c);
  CreateController(controller);
  DestroyController(controller);
  SetWarmStarting(flag);
  SetContinuousPhysics(flag);
  GetBodyCount();
  GetJointCount();
  GetContactCount();
  SetGravity(gravity);
  GetGravity();
  GetGroundBody();
  Step(dt, velocityIterations, positionIterations);
  ClearForces();
  DrawDebugData();
  QueryAABB(callback, aabb);
  QueryShape(callback, shape, transform);
  QueryPoint(callback, p);
  RayCast(callback, point1, point2);
  RayCastOne(point1, point2);
  RayCastAll(point1, point2);
  GetBodyList();
  GetJointList();
  GetContactList();
  IsLocked();
  Solve(step);
  SolveTOI(step);
  DrawJoint(joint);
  DrawShape(shape, xf, color);
  static b2World() => _b2World.b2World();
}

abstract class b2AABB {
  factory b2AABB() = _b2AABB;

  IsValid();
  GetCenter();
  GetExtents();
  Contains(aabb);
  RayCast(output, input);
  TestOverlap(other);
  Combine(aabb1, aabb2);
  static b2AABB() => _b2AABB.b2AABB();
  static Combine(aabb1, aabb2) => _b2AABB.Combine(aabb1, aabb2);
}

abstract class b2Bound {
  factory b2Bound() = _b2Bound;

  IsLower();
  IsUpper();
  Swap(b);
  static b2Bound() => _b2Bound.b2Bound();
}

abstract class b2BoundValues {
  factory b2BoundValues() = _b2BoundValues;

  b2BoundValues();
  static b2BoundValues() => _b2BoundValues.b2BoundValues();
}

abstract class b2DynamicTree {
  factory b2DynamicTree() = _b2DynamicTree;

  b2DynamicTree();
  CreateProxy(aabb, userData);
  DestroyProxy(proxy);
  MoveProxy(proxy, aabb, displacement);
  Rebalance(iterations);
  GetFatAABB(proxy);
  GetUserData(proxy);
  Query(callback, aabb);
  RayCast(callback, input);
  AllocateNode();
  FreeNode(node);
  InsertLeaf(leaf);
  RemoveLeaf(leaf);
  static b2DynamicTree() => _b2DynamicTree.b2DynamicTree();
}

abstract class b2DynamicTreeBroadPhase {
  factory b2DynamicTreeBroadPhase() = _b2DynamicTreeBroadPhase;

  CreateProxy(aabb, userData);
  DestroyProxy(proxy);
  MoveProxy(proxy, aabb, displacement);
  TestOverlap(proxyA, proxyB);
  GetUserData(proxy);
  GetFatAABB(proxy);
  GetProxyCount();
  UpdatePairs(callback);
  Query(callback, aabb);
  RayCast(callback, input);
  Validate();
  Rebalance(iterations);
  BufferMove(proxy);
  UnBufferMove(proxy);
  ComparePairs(pair1, pair2);
  static b2DynamicTreeBroadPhase() => _b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase();
}

abstract class b2DynamicTreeNode {
  factory b2DynamicTreeNode() = _b2DynamicTreeNode;

  IsLeaf();
  static b2DynamicTreeNode() => _b2DynamicTreeNode.b2DynamicTreeNode();
}

abstract class b2Manifold {
  factory b2Manifold() = _b2Manifold;

  b2Manifold();
  Reset();
  Set(m);
  Copy();
  static b2Manifold() => _b2Manifold.b2Manifold();
}

abstract class b2ManifoldPoint {
  factory b2ManifoldPoint() = _b2ManifoldPoint;

  b2ManifoldPoint();
  Reset();
  Set(m);
  static b2ManifoldPoint() => _b2ManifoldPoint.b2ManifoldPoint();
}

abstract class b2Point {
  factory b2Point() = _b2Point;

  Support(xf, vX, vY);
  GetFirstVertex(xf);
  static b2Point() => _b2Point.b2Point();
}

abstract class b2RayCastInput {
  factory b2RayCastInput(p1, p2, maxFraction) = _b2RayCastInput;

  b2RayCastInput(p1, p2, maxFraction);
  static b2RayCastInput() => _b2RayCastInput.b2RayCastInput();
}

abstract class b2Segment {
  factory b2Segment() = _b2Segment;

  TestSegment(lambda, normal, segment, maxLambda);
  Extend(aabb);
  ExtendForward(aabb);
  ExtendBackward(aabb);
  static b2Segment() => _b2Segment.b2Segment();
}

abstract class b2CircleShape extends b2Shape {
  factory b2CircleShape(radius) = _b2CircleShape;

  Copy();
  Set(other);
  TestPoint(transform, p);
  RayCast(output, input, transform);
  ComputeAABB(aabb, transform);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  GetLocalPosition();
  SetLocalPosition(position);
  GetRadius();
  SetRadius(radius);
  b2CircleShape(radius);
  static b2CircleShape() => _b2CircleShape.b2CircleShape();
}

abstract class b2EdgeChainDef {
  factory b2EdgeChainDef() = _b2EdgeChainDef;

  b2EdgeChainDef();
  static b2EdgeChainDef() => _b2EdgeChainDef.b2EdgeChainDef();
}

abstract class b2EdgeShape extends b2Shape {
  factory b2EdgeShape(v1, v2) = _b2EdgeShape;

  TestPoint(transform, p);
  RayCast(output, input, transform);
  ComputeAABB(aabb, transform);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  GetLength();
  GetVertex1();
  GetVertex2();
  GetCoreVertex1();
  GetCoreVertex2();
  GetNormalVector();
  GetDirectionVector();
  GetCorner1Vector();
  GetCorner2Vector();
  Corner1IsConvex();
  Corner2IsConvex();
  GetFirstVertex(xf);
  GetNextEdge();
  GetPrevEdge();
  Support(xf, dX, dY);
  b2EdgeShape(v1, v2);
  SetPrevEdge(edge, core, cornerDir, convex);
  SetNextEdge(edge, core, cornerDir, convex);
  static b2EdgeShape() => _b2EdgeShape.b2EdgeShape();
}

abstract class b2PolygonShape extends b2Shape {
  factory b2PolygonShape() = _b2PolygonShape;

  Copy();
  Set(other);
  SetAsArray(vertices, vertexCount);
  SetAsVector(vertices, vertexCount);
  SetAsBox(hx, hy);
  SetAsOrientedBox(hx, hy, center, angle);
  SetAsEdge(v1, v2);
  TestPoint(xf, p);
  RayCast(output, input, transform);
  ComputeAABB(aabb, xf);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  GetVertexCount();
  GetVertices();
  GetNormals();
  GetSupport(d);
  GetSupportVertex(d);
  Validate();
  b2PolygonShape();
  Reserve(count);
  static b2PolygonShape() => _b2PolygonShape.b2PolygonShape();
  static AsArray(vertices, vertexCount) => _b2PolygonShape.AsArray(vertices, vertexCount);
  static AsVector(vertices, vertexCount) => _b2PolygonShape.AsVector(vertices, vertexCount);
  static AsBox(hx, hy) => _b2PolygonShape.AsBox(hx, hy);
  static AsOrientedBox(hx, hy, center, angle) => _b2PolygonShape.AsOrientedBox(hx, hy, center, angle);
  static AsEdge(v1, v2) => _b2PolygonShape.AsEdge(v1, v2);
  static ComputeCentroid(vs, count) => _b2PolygonShape.ComputeCentroid(vs, count);
  static ComputeOBB(obb, vs, count) => _b2PolygonShape.ComputeOBB(obb, vs, count);
}

abstract class b2CircleContact extends b2Contact {
  factory b2CircleContact() = _b2CircleContact;

  Reset(fixtureA, fixtureB);
  Evaluate();
  static b2CircleContact() => _b2CircleContact.b2CircleContact();
  static Create(allocator) => _b2CircleContact.Create(allocator);
  static Destroy(contact, allocator) => _b2CircleContact.Destroy(contact, allocator);
}

abstract class b2ContactConstraint {
  factory b2ContactConstraint() = _b2ContactConstraint;

  b2ContactConstraint();
  static b2ContactConstraint() => _b2ContactConstraint.b2ContactConstraint();
}

abstract class b2ContactFactory {
  factory b2ContactFactory(allocator) = _b2ContactFactory;

  b2ContactFactory(allocator);
  AddType(createFcn, destroyFcn, type1, type2);
  InitializeRegisters();
  Create(fixtureA, fixtureB);
  Destroy(contact);
  static b2ContactFactory() => _b2ContactFactory.b2ContactFactory();
}

abstract class b2ContactSolver {
  factory b2ContactSolver() = _b2ContactSolver;

  b2ContactSolver();
  Initialize(step, contacts, contactCount, allocator);
  InitVelocityConstraints(step);
  SolveVelocityConstraints();
  FinalizeVelocityConstraints();
  SolvePositionConstraints(baumgarte);
  static b2ContactSolver() => _b2ContactSolver.b2ContactSolver();
}

abstract class b2EdgeAndCircleContact extends b2Contact {
  factory b2EdgeAndCircleContact() = _b2EdgeAndCircleContact;

  Reset(fixtureA, fixtureB);
  Evaluate();
  b2CollideEdgeAndCircle(manifold, edge, xf1, circle, xf2);
  static b2EdgeAndCircleContact() => _b2EdgeAndCircleContact.b2EdgeAndCircleContact();
  static Create(allocator) => _b2EdgeAndCircleContact.Create(allocator);
  static Destroy(contact, allocator) => _b2EdgeAndCircleContact.Destroy(contact, allocator);
}

abstract class b2NullContact extends b2Contact {
  factory b2NullContact() = _b2NullContact;

  b2NullContact();
  Evaluate();
  static b2NullContact() => _b2NullContact.b2NullContact();
}

abstract class b2PolyAndCircleContact extends b2Contact {
  factory b2PolyAndCircleContact() = _b2PolyAndCircleContact;

  Reset(fixtureA, fixtureB);
  Evaluate();
  static b2PolyAndCircleContact() => _b2PolyAndCircleContact.b2PolyAndCircleContact();
  static Create(allocator) => _b2PolyAndCircleContact.Create(allocator);
  static Destroy(contact, allocator) => _b2PolyAndCircleContact.Destroy(contact, allocator);
}

abstract class b2PolyAndEdgeContact extends b2Contact {
  factory b2PolyAndEdgeContact() = _b2PolyAndEdgeContact;

  Reset(fixtureA, fixtureB);
  Evaluate();
  b2CollidePolyAndEdge(manifold, polygon, xf1, edge, xf2);
  static b2PolyAndEdgeContact() => _b2PolyAndEdgeContact.b2PolyAndEdgeContact();
  static Create(allocator) => _b2PolyAndEdgeContact.Create(allocator);
  static Destroy(contact, allocator) => _b2PolyAndEdgeContact.Destroy(contact, allocator);
}

abstract class b2PolygonContact extends b2Contact {
  factory b2PolygonContact() = _b2PolygonContact;

  Reset(fixtureA, fixtureB);
  Evaluate();
  static b2PolygonContact() => _b2PolygonContact.b2PolygonContact();
  static Create(allocator) => _b2PolygonContact.Create(allocator);
  static Destroy(contact, allocator) => _b2PolygonContact.Destroy(contact, allocator);
}

abstract class b2BuoyancyController extends b2Controller {
  factory b2BuoyancyController() = _b2BuoyancyController;

  Step(step);
  Draw(debugDraw);
  static b2BuoyancyController() => _b2BuoyancyController.b2BuoyancyController();
}

abstract class b2ConstantAccelController extends b2Controller {
  factory b2ConstantAccelController() = _b2ConstantAccelController;

  Step(step);
  static b2ConstantAccelController() => _b2ConstantAccelController.b2ConstantAccelController();
}

abstract class b2ConstantForceController extends b2Controller {
  factory b2ConstantForceController() = _b2ConstantForceController;

  Step(step);
  static b2ConstantForceController() => _b2ConstantForceController.b2ConstantForceController();
}

abstract class b2GravityController extends b2Controller {
  factory b2GravityController() = _b2GravityController;

  Step(step);
  static b2GravityController() => _b2GravityController.b2GravityController();
}

abstract class b2TensorDampingController extends b2Controller {
  factory b2TensorDampingController() = _b2TensorDampingController;

  SetAxisAligned(xDamping, yDamping);
  Step(step);
  static b2TensorDampingController() => _b2TensorDampingController.b2TensorDampingController();
}

abstract class b2ControllerEdge {
  factory b2ControllerEdge() = _b2ControllerEdge;

  static b2ControllerEdge() => _b2ControllerEdge.b2ControllerEdge();
}

abstract class Vector_a2j_Number {
  factory Vector_a2j_Number(length) = _Vector_a2j_Number;

}

abstract class b2ContactRegister {
  factory b2ContactRegister() = _b2ContactRegister;

  static b2ContactRegister() => _b2ContactRegister.b2ContactRegister();
}

abstract class b2ContactConstraintPoint {
  factory b2ContactConstraintPoint() = _b2ContactConstraintPoint;

  static b2ContactConstraintPoint() => _b2ContactConstraintPoint.b2ContactConstraintPoint();
}

abstract class b2ContactEdge {
  factory b2ContactEdge() = _b2ContactEdge;

  static b2ContactEdge() => _b2ContactEdge.b2ContactEdge();
}

abstract class b2MassData {
  factory b2MassData() = _b2MassData;

  static b2MassData() => _b2MassData.b2MassData();
}

abstract class b2DynamicTreePair {
  factory b2DynamicTreePair() = _b2DynamicTreePair;

  static b2DynamicTreePair() => _b2DynamicTreePair.b2DynamicTreePair();
}

abstract class b2RayCastOutput {
  factory b2RayCastOutput() = _b2RayCastOutput;

  static b2RayCastOutput() => _b2RayCastOutput.b2RayCastOutput();
}

abstract class b2JointEdge {
  factory b2JointEdge() = _b2JointEdge;

  static b2JointEdge() => _b2JointEdge.b2JointEdge();
}

class _b2TimeStep implements b2TimeStep {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2TimeStep"];
  _b2TimeStep() : jsvalue = new JsObject(_constructor, []);

  Set(step) => jsvalue.callMethod(r"Set", [step]);
  static b2TimeStep() => _constructor.callMethod(r"b2TimeStep", []);
}

class _b2Transform implements b2Transform {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Transform"];
  _b2Transform(pos, r) : jsvalue = new JsObject(_constructor, [pos, r]);

  b2Transform(pos, r) => jsvalue.callMethod(r"b2Transform", [pos, r]);
  Initialize(pos, r) => jsvalue.callMethod(r"Initialize", [pos, r]);
  SetIdentity() => jsvalue.callMethod(r"SetIdentity", []);
  Set(x) => jsvalue.callMethod(r"Set", [x]);
  GetAngle() => jsvalue.callMethod(r"GetAngle", []);
  static b2Transform() => _constructor.callMethod(r"b2Transform", []);
}

class _b2Sweep implements b2Sweep {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Sweep"];
  _b2Sweep() : jsvalue = new JsObject(_constructor, []);

  Set(other) => jsvalue.callMethod(r"Set", [other]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  GetTransform(xf, alpha) => jsvalue.callMethod(r"GetTransform", [xf, alpha]);
  Advance(t) => jsvalue.callMethod(r"Advance", [t]);
  static b2Sweep() => _constructor.callMethod(r"b2Sweep", []);
}

class _b2Color implements b2Color {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["b2Color"];
  _b2Color(rr, gg, bb) : jsvalue = new JsObject(_constructor, [rr, gg, bb]);

  b2Color(rr, gg, bb) => jsvalue.callMethod(r"b2Color", [rr, gg, bb]);
  Set(rr, gg, bb) => jsvalue.callMethod(r"Set", [rr, gg, bb]);
  static b2Color() => _constructor.callMethod(r"b2Color", []);
}

class _b2Controller implements b2Controller {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2Controller"];
  _b2Controller() : jsvalue = new JsObject(_constructor, []);
  _b2Controller.withValue(value) : jsvalue = value;

  Step(step) => jsvalue.callMethod(r"Step", [step]);
  Draw(debugDraw) => jsvalue.callMethod(r"Draw", [debugDraw]);
  AddBody(body) => jsvalue.callMethod(r"AddBody", [body]);
  RemoveBody(body) => jsvalue.callMethod(r"RemoveBody", [body]);
  Clear() => jsvalue.callMethod(r"Clear", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetWorld() => jsvalue.callMethod(r"GetWorld", []);
  GetBodyList() => jsvalue.callMethod(r"GetBodyList", []);
  static b2Controller() => _constructor.callMethod(r"b2Controller", []);
}

class _b2Vec2 implements b2Vec2 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Vec2"];
  _b2Vec2(x_, y_) : jsvalue = new JsObject(_constructor, [x_, y_]);

  b2Vec2(x_, y_) => jsvalue.callMethod(r"b2Vec2", [x_, y_]);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Set(x_, y_) => jsvalue.callMethod(r"Set", [x_, y_]);
  SetV(v) => jsvalue.callMethod(r"SetV", [v]);
  GetNegative() => jsvalue.callMethod(r"GetNegative", []);
  NegativeSelf() => jsvalue.callMethod(r"NegativeSelf", []);
  Copy() => jsvalue.callMethod(r"Copy", []);
  Add(v) => jsvalue.callMethod(r"Add", [v]);
  Subtract(v) => jsvalue.callMethod(r"Subtract", [v]);
  Multiply(a) => jsvalue.callMethod(r"Multiply", [a]);
  MulM(A) => jsvalue.callMethod(r"MulM", [A]);
  MulTM(A) => jsvalue.callMethod(r"MulTM", [A]);
  CrossVF(s) => jsvalue.callMethod(r"CrossVF", [s]);
  CrossFV(s) => jsvalue.callMethod(r"CrossFV", [s]);
  MinV(b) => jsvalue.callMethod(r"MinV", [b]);
  MaxV(b) => jsvalue.callMethod(r"MaxV", [b]);
  Abs() => jsvalue.callMethod(r"Abs", []);
  Length() => jsvalue.callMethod(r"Length", []);
  LengthSquared() => jsvalue.callMethod(r"LengthSquared", []);
  Normalize() => jsvalue.callMethod(r"Normalize", []);
  IsValid() => jsvalue.callMethod(r"IsValid", []);
  static b2Vec2() => _constructor.callMethod(r"b2Vec2", []);
  static Make(x_, y_) => _constructor.callMethod(r"Make", [x_, y_]);
}

class _b2Contact implements b2Contact {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2Contact"];
  _b2Contact() : jsvalue = new JsObject(_constructor, []);
  _b2Contact.withValue(value) : jsvalue = value;

  GetManifold() => jsvalue.callMethod(r"GetManifold", []);
  GetWorldManifold(worldManifold) => jsvalue.callMethod(r"GetWorldManifold", [worldManifold]);
  IsTouching() => jsvalue.callMethod(r"IsTouching", []);
  IsContinuous() => jsvalue.callMethod(r"IsContinuous", []);
  SetSensor(sensor) => jsvalue.callMethod(r"SetSensor", [sensor]);
  IsSensor() => jsvalue.callMethod(r"IsSensor", []);
  SetEnabled(flag) => jsvalue.callMethod(r"SetEnabled", [flag]);
  IsEnabled() => jsvalue.callMethod(r"IsEnabled", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetFixtureA() => jsvalue.callMethod(r"GetFixtureA", []);
  GetFixtureB() => jsvalue.callMethod(r"GetFixtureB", []);
  FlagForFiltering() => jsvalue.callMethod(r"FlagForFiltering", []);
  b2Contact() => jsvalue.callMethod(r"b2Contact", []);
  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [fixtureA, fixtureB]);
  Update(listener) => jsvalue.callMethod(r"Update", [listener]);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  ComputeTOI(sweepA, sweepB) => jsvalue.callMethod(r"ComputeTOI", [sweepA, sweepB]);
  static b2Contact() => _constructor.callMethod(r"b2Contact", []);
}

class _b2WorldManifold implements b2WorldManifold {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2WorldManifold"];
  _b2WorldManifold() : jsvalue = new JsObject(_constructor, []);

  b2WorldManifold() => jsvalue.callMethod(r"b2WorldManifold", []);
  Initialize(manifold, xfA, radiusA, xfB, radiusB) => jsvalue.callMethod(r"Initialize", [manifold, xfA, radiusA, xfB, radiusB]);
  static b2WorldManifold() => _constructor.callMethod(r"b2WorldManifold", []);
}

class _b2PositionSolverManifold implements b2PositionSolverManifold {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PositionSolverManifold"];
  _b2PositionSolverManifold() : jsvalue = new JsObject(_constructor, []);

  b2PositionSolverManifold() => jsvalue.callMethod(r"b2PositionSolverManifold", []);
  Initialize(cc) => jsvalue.callMethod(r"Initialize", [cc]);
  static b2PositionSolverManifold() => _constructor.callMethod(r"b2PositionSolverManifold", []);
}

class _b2TOIInput implements b2TOIInput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2TOIInput"];
  _b2TOIInput() : jsvalue = new JsObject(_constructor, []);

  static b2TOIInput() => _constructor.callMethod(r"b2TOIInput", []);
}

class _b2DistanceProxy implements b2DistanceProxy {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DistanceProxy"];
  _b2DistanceProxy() : jsvalue = new JsObject(_constructor, []);

  Set(shape) => jsvalue.callMethod(r"Set", [shape]);
  GetSupport(d) => jsvalue.callMethod(r"GetSupport", [d]);
  GetSupportVertex(d) => jsvalue.callMethod(r"GetSupportVertex", [d]);
  GetVertexCount() => jsvalue.callMethod(r"GetVertexCount", []);
  GetVertex(index) => jsvalue.callMethod(r"GetVertex", [index]);
  static b2DistanceProxy() => _constructor.callMethod(r"b2DistanceProxy", []);
}

class _b2Shape implements b2Shape {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2Shape"];
  _b2Shape() : jsvalue = new JsObject(_constructor, []);
  _b2Shape.withValue(value) : jsvalue = value;

  Copy() => jsvalue.callMethod(r"Copy", []);
  Set(other) => jsvalue.callMethod(r"Set", [other]);
  GetType() => jsvalue.callMethod(r"GetType", []);
  TestPoint(xf, p) => jsvalue.callMethod(r"TestPoint", [xf, p]);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [output, input, transform]);
  ComputeAABB(aabb, xf) => jsvalue.callMethod(r"ComputeAABB", [aabb, xf]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [massData, density]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [normal, offset, xf, c]);
  b2Shape() => jsvalue.callMethod(r"b2Shape", []);
  static b2Shape() => _constructor.callMethod(r"b2Shape", []);
  static TestOverlap(shape1, transform1, shape2, transform2) => _constructor.callMethod(r"TestOverlap", [shape1, transform1, shape2, transform2]);
}

class _b2Mat22 implements b2Mat22 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Mat22"];
  _b2Mat22() : jsvalue = new JsObject(_constructor, []);

  b2Mat22() => jsvalue.callMethod(r"b2Mat22", []);
  Set(angle) => jsvalue.callMethod(r"Set", [angle]);
  SetVV(c1, c2) => jsvalue.callMethod(r"SetVV", [c1, c2]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  SetM(m) => jsvalue.callMethod(r"SetM", [m]);
  AddM(m) => jsvalue.callMethod(r"AddM", [m]);
  SetIdentity() => jsvalue.callMethod(r"SetIdentity", []);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  GetAngle() => jsvalue.callMethod(r"GetAngle", []);
  GetInverse(out) => jsvalue.callMethod(r"GetInverse", [out]);
  Solve(out, bX, bY) => jsvalue.callMethod(r"Solve", [out, bX, bY]);
  Abs() => jsvalue.callMethod(r"Abs", []);
  static b2Mat22() => _constructor.callMethod(r"b2Mat22", []);
  static FromAngle(angle) => _constructor.callMethod(r"FromAngle", [angle]);
  static FromVV(c1, c2) => _constructor.callMethod(r"FromVV", [c1, c2]);
}

class _b2SimplexCache implements b2SimplexCache {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2SimplexCache"];
  _b2SimplexCache() : jsvalue = new JsObject(_constructor, []);

  static b2SimplexCache() => _constructor.callMethod(r"b2SimplexCache", []);
}

class _b2DistanceInput implements b2DistanceInput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DistanceInput"];
  _b2DistanceInput() : jsvalue = new JsObject(_constructor, []);

  static b2DistanceInput() => _constructor.callMethod(r"b2DistanceInput", []);
}

class _b2SeparationFunction implements b2SeparationFunction {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2SeparationFunction"];
  _b2SeparationFunction() : jsvalue = new JsObject(_constructor, []);

  Initialize(cache, proxyA, transformA, proxyB, transformB) => jsvalue.callMethod(r"Initialize", [cache, proxyA, transformA, proxyB, transformB]);
  Evaluate(transformA, transformB) => jsvalue.callMethod(r"Evaluate", [transformA, transformB]);
  static b2SeparationFunction() => _constructor.callMethod(r"b2SeparationFunction", []);
}

class _b2DistanceOutput implements b2DistanceOutput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DistanceOutput"];
  _b2DistanceOutput() : jsvalue = new JsObject(_constructor, []);

  static b2DistanceOutput() => _constructor.callMethod(r"b2DistanceOutput", []);
}

class _b2Simplex implements b2Simplex {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Simplex"];
  _b2Simplex() : jsvalue = new JsObject(_constructor, []);

  b2Simplex() => jsvalue.callMethod(r"b2Simplex", []);
  ReadCache(cache, proxyA, transformA, proxyB, transformB) => jsvalue.callMethod(r"ReadCache", [cache, proxyA, transformA, proxyB, transformB]);
  WriteCache(cache) => jsvalue.callMethod(r"WriteCache", [cache]);
  GetSearchDirection() => jsvalue.callMethod(r"GetSearchDirection", []);
  GetClosestPoint() => jsvalue.callMethod(r"GetClosestPoint", []);
  GetWitnessPoints(pA, pB) => jsvalue.callMethod(r"GetWitnessPoints", [pA, pB]);
  GetMetric() => jsvalue.callMethod(r"GetMetric", []);
  Solve2() => jsvalue.callMethod(r"Solve2", []);
  Solve3() => jsvalue.callMethod(r"Solve3", []);
  static b2Simplex() => _constructor.callMethod(r"b2Simplex", []);
}

class _b2SimplexVertex implements b2SimplexVertex {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2SimplexVertex"];
  _b2SimplexVertex() : jsvalue = new JsObject(_constructor, []);

  Set(other) => jsvalue.callMethod(r"Set", [other]);
  static b2SimplexVertex() => _constructor.callMethod(r"b2SimplexVertex", []);
}

class _ClipVertex implements ClipVertex {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["ClipVertex"];
  _ClipVertex() : jsvalue = new JsObject(_constructor, []);

  Set(other) => jsvalue.callMethod(r"Set", [other]);
  static ClipVertex() => _constructor.callMethod(r"ClipVertex", []);
}

class _b2ContactID implements b2ContactID {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2ContactID"];
  _b2ContactID() : jsvalue = new JsObject(_constructor, []);

  b2ContactID() => jsvalue.callMethod(r"b2ContactID", []);
  Set(id) => jsvalue.callMethod(r"Set", [id]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  static b2ContactID() => _constructor.callMethod(r"b2ContactID", []);
}

class _Features implements Features {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Features"];
  _Features() : jsvalue = new JsObject(_constructor, []);

  static Features() => _constructor.callMethod(r"Features", []);
}

class _b2ContactImpulse implements b2ContactImpulse {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactImpulse"];
  _b2ContactImpulse() : jsvalue = new JsObject(_constructor, []);

  static b2ContactImpulse() => _constructor.callMethod(r"b2ContactImpulse", []);
}

class _b2ContactPoint implements b2ContactPoint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2ContactPoint"];
  _b2ContactPoint() : jsvalue = new JsObject(_constructor, []);

  static b2ContactPoint() => _constructor.callMethod(r"b2ContactPoint", []);
}

class _b2ContactListener implements b2ContactListener {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactListener"];
  _b2ContactListener() : jsvalue = new JsObject(_constructor, []);

  BeginContact(contact) => jsvalue.callMethod(r"BeginContact", [contact]);
  EndContact(contact) => jsvalue.callMethod(r"EndContact", [contact]);
  PreSolve(contact, oldManifold) => jsvalue.callMethod(r"PreSolve", [contact, oldManifold]);
  PostSolve(contact, impulse) => jsvalue.callMethod(r"PostSolve", [contact, impulse]);
  static b2ContactListener() => _constructor.callMethod(r"b2ContactListener", []);
}

class _b2ContactFilter implements b2ContactFilter {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactFilter"];
  _b2ContactFilter() : jsvalue = new JsObject(_constructor, []);

  ShouldCollide(fixtureA, fixtureB) => jsvalue.callMethod(r"ShouldCollide", [fixtureA, fixtureB]);
  RayCollide(userData, fixture) => jsvalue.callMethod(r"RayCollide", [userData, fixture]);
  static b2ContactFilter() => _constructor.callMethod(r"b2ContactFilter", []);
}

class _b2JointDef implements b2JointDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2JointDef"];
  _b2JointDef() : jsvalue = new JsObject(_constructor, []);
  _b2JointDef.withValue(value) : jsvalue = value;

  b2JointDef() => jsvalue.callMethod(r"b2JointDef", []);
  static b2JointDef() => _constructor.callMethod(r"b2JointDef", []);
}

class _b2Joint implements b2Joint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2Joint"];
  _b2Joint(def) : jsvalue = new JsObject(_constructor, [def]);
  _b2Joint.withValue(value) : jsvalue = value;

  GetType() => jsvalue.callMethod(r"GetType", []);
  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetBodyA() => jsvalue.callMethod(r"GetBodyA", []);
  GetBodyB() => jsvalue.callMethod(r"GetBodyB", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetUserData() => jsvalue.callMethod(r"GetUserData", []);
  SetUserData(data) => jsvalue.callMethod(r"SetUserData", [data]);
  IsActive() => jsvalue.callMethod(r"IsActive", []);
  b2Joint(def) => jsvalue.callMethod(r"b2Joint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  FinalizeVelocityConstraints() => jsvalue.callMethod(r"FinalizeVelocityConstraints", []);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2Joint() => _constructor.callMethod(r"b2Joint", []);
  static Create(def, allocator) => _constructor.callMethod(r"Create", [def, allocator]);
  static Destroy(joint, allocator) => _constructor.callMethod(r"Destroy", [joint, allocator]);
}

class _b2Mat33 implements b2Mat33 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Mat33"];
  _b2Mat33(c1, c2, c3) : jsvalue = new JsObject(_constructor, [c1, c2, c3]);

  b2Mat33(c1, c2, c3) => jsvalue.callMethod(r"b2Mat33", [c1, c2, c3]);
  SetVVV(c1, c2, c3) => jsvalue.callMethod(r"SetVVV", [c1, c2, c3]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  SetM(m) => jsvalue.callMethod(r"SetM", [m]);
  AddM(m) => jsvalue.callMethod(r"AddM", [m]);
  SetIdentity() => jsvalue.callMethod(r"SetIdentity", []);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Solve22(out, bX, bY) => jsvalue.callMethod(r"Solve22", [out, bX, bY]);
  Solve33(out, bX, bY, bZ) => jsvalue.callMethod(r"Solve33", [out, bX, bY, bZ]);
  static b2Mat33() => _constructor.callMethod(r"b2Mat33", []);
}

class _b2Vec3 implements b2Vec3 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Vec3"];
  _b2Vec3(x, y, z) : jsvalue = new JsObject(_constructor, [x, y, z]);

  b2Vec3(x, y, z) => jsvalue.callMethod(r"b2Vec3", [x, y, z]);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Set(x, y, z) => jsvalue.callMethod(r"Set", [x, y, z]);
  SetV(v) => jsvalue.callMethod(r"SetV", [v]);
  GetNegative() => jsvalue.callMethod(r"GetNegative", []);
  NegativeSelf() => jsvalue.callMethod(r"NegativeSelf", []);
  Copy() => jsvalue.callMethod(r"Copy", []);
  Add(v) => jsvalue.callMethod(r"Add", [v]);
  Subtract(v) => jsvalue.callMethod(r"Subtract", [v]);
  Multiply(a) => jsvalue.callMethod(r"Multiply", [a]);
  static b2Vec3() => _constructor.callMethod(r"b2Vec3", []);
}

class _b2DistanceJoint extends _b2Joint implements b2DistanceJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2DistanceJoint"];
  _b2DistanceJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetLength() => jsvalue.callMethod(r"GetLength", []);
  SetLength(length) => jsvalue.callMethod(r"SetLength", [length]);
  GetFrequency() => jsvalue.callMethod(r"GetFrequency", []);
  SetFrequency(hz) => jsvalue.callMethod(r"SetFrequency", [hz]);
  GetDampingRatio() => jsvalue.callMethod(r"GetDampingRatio", []);
  SetDampingRatio(ratio) => jsvalue.callMethod(r"SetDampingRatio", [ratio]);
  b2DistanceJoint(def) => jsvalue.callMethod(r"b2DistanceJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2DistanceJoint() => _constructor.callMethod(r"b2DistanceJoint", []);
}

class _b2DistanceJointDef extends _b2JointDef implements b2DistanceJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2DistanceJointDef"];
  _b2DistanceJointDef() : super.withValue(new JsObject(_constructor, []));

  b2DistanceJointDef() => jsvalue.callMethod(r"b2DistanceJointDef", []);
  Initialize(bA, bB, anchorA, anchorB) => jsvalue.callMethod(r"Initialize", [bA, bB, anchorA, anchorB]);
  static b2DistanceJointDef() => _constructor.callMethod(r"b2DistanceJointDef", []);
}

class _b2FrictionJoint extends _b2Joint implements b2FrictionJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2FrictionJoint"];
  _b2FrictionJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  SetMaxForce(force) => jsvalue.callMethod(r"SetMaxForce", [force]);
  GetMaxForce() => jsvalue.callMethod(r"GetMaxForce", []);
  SetMaxTorque(torque) => jsvalue.callMethod(r"SetMaxTorque", [torque]);
  GetMaxTorque() => jsvalue.callMethod(r"GetMaxTorque", []);
  b2FrictionJoint(def) => jsvalue.callMethod(r"b2FrictionJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2FrictionJoint() => _constructor.callMethod(r"b2FrictionJoint", []);
}

class _b2FrictionJointDef extends _b2JointDef implements b2FrictionJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2FrictionJointDef"];
  _b2FrictionJointDef() : super.withValue(new JsObject(_constructor, []));

  b2FrictionJointDef() => jsvalue.callMethod(r"b2FrictionJointDef", []);
  Initialize(bA, bB, anchor) => jsvalue.callMethod(r"Initialize", [bA, bB, anchor]);
  static b2FrictionJointDef() => _constructor.callMethod(r"b2FrictionJointDef", []);
}

class _b2GearJoint extends _b2Joint implements b2GearJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2GearJoint"];
  _b2GearJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetRatio() => jsvalue.callMethod(r"GetRatio", []);
  SetRatio(ratio) => jsvalue.callMethod(r"SetRatio", [ratio]);
  b2GearJoint(def) => jsvalue.callMethod(r"b2GearJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2GearJoint() => _constructor.callMethod(r"b2GearJoint", []);
}

class _b2GearJointDef extends _b2JointDef implements b2GearJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2GearJointDef"];
  _b2GearJointDef() : super.withValue(new JsObject(_constructor, []));

  b2GearJointDef() => jsvalue.callMethod(r"b2GearJointDef", []);
  static b2GearJointDef() => _constructor.callMethod(r"b2GearJointDef", []);
}

class _b2Jacobian implements b2Jacobian {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2Jacobian"];
  _b2Jacobian() : jsvalue = new JsObject(_constructor, []);

  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Set(x1, a1, x2, a2) => jsvalue.callMethod(r"Set", [x1, a1, x2, a2]);
  Compute(x1, a1, x2, a2) => jsvalue.callMethod(r"Compute", [x1, a1, x2, a2]);
  static b2Jacobian() => _constructor.callMethod(r"b2Jacobian", []);
}

class _b2LineJoint extends _b2Joint implements b2LineJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2LineJoint"];
  _b2LineJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetJointTranslation() => jsvalue.callMethod(r"GetJointTranslation", []);
  GetJointSpeed() => jsvalue.callMethod(r"GetJointSpeed", []);
  IsLimitEnabled() => jsvalue.callMethod(r"IsLimitEnabled", []);
  EnableLimit(flag) => jsvalue.callMethod(r"EnableLimit", [flag]);
  GetLowerLimit() => jsvalue.callMethod(r"GetLowerLimit", []);
  GetUpperLimit() => jsvalue.callMethod(r"GetUpperLimit", []);
  SetLimits(lower, upper) => jsvalue.callMethod(r"SetLimits", [lower, upper]);
  IsMotorEnabled() => jsvalue.callMethod(r"IsMotorEnabled", []);
  EnableMotor(flag) => jsvalue.callMethod(r"EnableMotor", [flag]);
  SetMotorSpeed(speed) => jsvalue.callMethod(r"SetMotorSpeed", [speed]);
  GetMotorSpeed() => jsvalue.callMethod(r"GetMotorSpeed", []);
  SetMaxMotorForce(force) => jsvalue.callMethod(r"SetMaxMotorForce", [force]);
  GetMaxMotorForce() => jsvalue.callMethod(r"GetMaxMotorForce", []);
  GetMotorForce() => jsvalue.callMethod(r"GetMotorForce", []);
  b2LineJoint(def) => jsvalue.callMethod(r"b2LineJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2LineJoint() => _constructor.callMethod(r"b2LineJoint", []);
}

class _b2LineJointDef extends _b2JointDef implements b2LineJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2LineJointDef"];
  _b2LineJointDef() : super.withValue(new JsObject(_constructor, []));

  b2LineJointDef() => jsvalue.callMethod(r"b2LineJointDef", []);
  Initialize(bA, bB, anchor, axis) => jsvalue.callMethod(r"Initialize", [bA, bB, anchor, axis]);
  static b2LineJointDef() => _constructor.callMethod(r"b2LineJointDef", []);
}

class _b2MouseJoint extends _b2Joint implements b2MouseJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2MouseJoint"];
  _b2MouseJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetTarget() => jsvalue.callMethod(r"GetTarget", []);
  SetTarget(target) => jsvalue.callMethod(r"SetTarget", [target]);
  GetMaxForce() => jsvalue.callMethod(r"GetMaxForce", []);
  SetMaxForce(maxForce) => jsvalue.callMethod(r"SetMaxForce", [maxForce]);
  GetFrequency() => jsvalue.callMethod(r"GetFrequency", []);
  SetFrequency(hz) => jsvalue.callMethod(r"SetFrequency", [hz]);
  GetDampingRatio() => jsvalue.callMethod(r"GetDampingRatio", []);
  SetDampingRatio(ratio) => jsvalue.callMethod(r"SetDampingRatio", [ratio]);
  b2MouseJoint(def) => jsvalue.callMethod(r"b2MouseJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2MouseJoint() => _constructor.callMethod(r"b2MouseJoint", []);
}

class _b2MouseJointDef extends _b2JointDef implements b2MouseJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2MouseJointDef"];
  _b2MouseJointDef() : super.withValue(new JsObject(_constructor, []));

  b2MouseJointDef() => jsvalue.callMethod(r"b2MouseJointDef", []);
  static b2MouseJointDef() => _constructor.callMethod(r"b2MouseJointDef", []);
}

class _b2PrismaticJoint extends _b2Joint implements b2PrismaticJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2PrismaticJoint"];
  _b2PrismaticJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetJointTranslation() => jsvalue.callMethod(r"GetJointTranslation", []);
  GetJointSpeed() => jsvalue.callMethod(r"GetJointSpeed", []);
  IsLimitEnabled() => jsvalue.callMethod(r"IsLimitEnabled", []);
  EnableLimit(flag) => jsvalue.callMethod(r"EnableLimit", [flag]);
  GetLowerLimit() => jsvalue.callMethod(r"GetLowerLimit", []);
  GetUpperLimit() => jsvalue.callMethod(r"GetUpperLimit", []);
  SetLimits(lower, upper) => jsvalue.callMethod(r"SetLimits", [lower, upper]);
  IsMotorEnabled() => jsvalue.callMethod(r"IsMotorEnabled", []);
  EnableMotor(flag) => jsvalue.callMethod(r"EnableMotor", [flag]);
  SetMotorSpeed(speed) => jsvalue.callMethod(r"SetMotorSpeed", [speed]);
  GetMotorSpeed() => jsvalue.callMethod(r"GetMotorSpeed", []);
  SetMaxMotorForce(force) => jsvalue.callMethod(r"SetMaxMotorForce", [force]);
  GetMotorForce() => jsvalue.callMethod(r"GetMotorForce", []);
  b2PrismaticJoint(def) => jsvalue.callMethod(r"b2PrismaticJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2PrismaticJoint() => _constructor.callMethod(r"b2PrismaticJoint", []);
}

class _b2PrismaticJointDef extends _b2JointDef implements b2PrismaticJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2PrismaticJointDef"];
  _b2PrismaticJointDef() : super.withValue(new JsObject(_constructor, []));

  b2PrismaticJointDef() => jsvalue.callMethod(r"b2PrismaticJointDef", []);
  Initialize(bA, bB, anchor, axis) => jsvalue.callMethod(r"Initialize", [bA, bB, anchor, axis]);
  static b2PrismaticJointDef() => _constructor.callMethod(r"b2PrismaticJointDef", []);
}

class _b2PulleyJoint extends _b2Joint implements b2PulleyJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2PulleyJoint"];
  _b2PulleyJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetGroundAnchorA() => jsvalue.callMethod(r"GetGroundAnchorA", []);
  GetGroundAnchorB() => jsvalue.callMethod(r"GetGroundAnchorB", []);
  GetLength1() => jsvalue.callMethod(r"GetLength1", []);
  GetLength2() => jsvalue.callMethod(r"GetLength2", []);
  GetRatio() => jsvalue.callMethod(r"GetRatio", []);
  b2PulleyJoint(def) => jsvalue.callMethod(r"b2PulleyJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2PulleyJoint() => _constructor.callMethod(r"b2PulleyJoint", []);
}

class _b2PulleyJointDef extends _b2JointDef implements b2PulleyJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2PulleyJointDef"];
  _b2PulleyJointDef() : super.withValue(new JsObject(_constructor, []));

  b2PulleyJointDef() => jsvalue.callMethod(r"b2PulleyJointDef", []);
  Initialize(bA, bB, gaA, gaB, anchorA, anchorB, r) => jsvalue.callMethod(r"Initialize", [bA, bB, gaA, gaB, anchorA, anchorB, r]);
  static b2PulleyJointDef() => _constructor.callMethod(r"b2PulleyJointDef", []);
}

class _b2RevoluteJoint extends _b2Joint implements b2RevoluteJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2RevoluteJoint"];
  _b2RevoluteJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  GetJointAngle() => jsvalue.callMethod(r"GetJointAngle", []);
  GetJointSpeed() => jsvalue.callMethod(r"GetJointSpeed", []);
  IsLimitEnabled() => jsvalue.callMethod(r"IsLimitEnabled", []);
  EnableLimit(flag) => jsvalue.callMethod(r"EnableLimit", [flag]);
  GetLowerLimit() => jsvalue.callMethod(r"GetLowerLimit", []);
  GetUpperLimit() => jsvalue.callMethod(r"GetUpperLimit", []);
  SetLimits(lower, upper) => jsvalue.callMethod(r"SetLimits", [lower, upper]);
  IsMotorEnabled() => jsvalue.callMethod(r"IsMotorEnabled", []);
  EnableMotor(flag) => jsvalue.callMethod(r"EnableMotor", [flag]);
  SetMotorSpeed(speed) => jsvalue.callMethod(r"SetMotorSpeed", [speed]);
  GetMotorSpeed() => jsvalue.callMethod(r"GetMotorSpeed", []);
  SetMaxMotorTorque(torque) => jsvalue.callMethod(r"SetMaxMotorTorque", [torque]);
  GetMotorTorque() => jsvalue.callMethod(r"GetMotorTorque", []);
  b2RevoluteJoint(def) => jsvalue.callMethod(r"b2RevoluteJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2RevoluteJoint() => _constructor.callMethod(r"b2RevoluteJoint", []);
}

class _b2RevoluteJointDef extends _b2JointDef implements b2RevoluteJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2RevoluteJointDef"];
  _b2RevoluteJointDef() : super.withValue(new JsObject(_constructor, []));

  b2RevoluteJointDef() => jsvalue.callMethod(r"b2RevoluteJointDef", []);
  Initialize(bA, bB, anchor) => jsvalue.callMethod(r"Initialize", [bA, bB, anchor]);
  static b2RevoluteJointDef() => _constructor.callMethod(r"b2RevoluteJointDef", []);
}

class _b2WeldJoint extends _b2Joint implements b2WeldJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2WeldJoint"];
  _b2WeldJoint(def) : super.withValue(new JsObject(_constructor, [def]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [inv_dt]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [inv_dt]);
  b2WeldJoint(def) => jsvalue.callMethod(r"b2WeldJoint", [def]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [step]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2WeldJoint() => _constructor.callMethod(r"b2WeldJoint", []);
}

class _b2WeldJointDef extends _b2JointDef implements b2WeldJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2WeldJointDef"];
  _b2WeldJointDef() : super.withValue(new JsObject(_constructor, []));

  b2WeldJointDef() => jsvalue.callMethod(r"b2WeldJointDef", []);
  Initialize(bA, bB, anchor) => jsvalue.callMethod(r"Initialize", [bA, bB, anchor]);
  static b2WeldJointDef() => _constructor.callMethod(r"b2WeldJointDef", []);
}

class _b2Body implements b2Body {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2Body"];
  _b2Body(bd, world) : jsvalue = new JsObject(_constructor, [bd, world]);

  connectEdges(s1, s2, angle1) => jsvalue.callMethod(r"connectEdges", [s1, s2, angle1]);
  CreateFixture(def) => jsvalue.callMethod(r"CreateFixture", [def]);
  CreateFixture2(shape, density) => jsvalue.callMethod(r"CreateFixture2", [shape, density]);
  DestroyFixture(fixture) => jsvalue.callMethod(r"DestroyFixture", [fixture]);
  SetPositionAndAngle(position, angle) => jsvalue.callMethod(r"SetPositionAndAngle", [position, angle]);
  SetTransform(xf) => jsvalue.callMethod(r"SetTransform", [xf]);
  GetTransform() => jsvalue.callMethod(r"GetTransform", []);
  GetPosition() => jsvalue.callMethod(r"GetPosition", []);
  SetPosition(position) => jsvalue.callMethod(r"SetPosition", [position]);
  GetAngle() => jsvalue.callMethod(r"GetAngle", []);
  SetAngle(angle) => jsvalue.callMethod(r"SetAngle", [angle]);
  GetWorldCenter() => jsvalue.callMethod(r"GetWorldCenter", []);
  GetLocalCenter() => jsvalue.callMethod(r"GetLocalCenter", []);
  SetLinearVelocity(v) => jsvalue.callMethod(r"SetLinearVelocity", [v]);
  GetLinearVelocity() => jsvalue.callMethod(r"GetLinearVelocity", []);
  SetAngularVelocity(omega) => jsvalue.callMethod(r"SetAngularVelocity", [omega]);
  GetAngularVelocity() => jsvalue.callMethod(r"GetAngularVelocity", []);
  GetDefinition() => jsvalue.callMethod(r"GetDefinition", []);
  ApplyForce(force, point) => jsvalue.callMethod(r"ApplyForce", [force, point]);
  ApplyTorque(torque) => jsvalue.callMethod(r"ApplyTorque", [torque]);
  ApplyImpulse(impulse, point) => jsvalue.callMethod(r"ApplyImpulse", [impulse, point]);
  Split(callback) => jsvalue.callMethod(r"Split", [callback]);
  Merge(other) => jsvalue.callMethod(r"Merge", [other]);
  GetMass() => jsvalue.callMethod(r"GetMass", []);
  GetInertia() => jsvalue.callMethod(r"GetInertia", []);
  GetMassData(data) => jsvalue.callMethod(r"GetMassData", [data]);
  SetMassData(massData) => jsvalue.callMethod(r"SetMassData", [massData]);
  ResetMassData() => jsvalue.callMethod(r"ResetMassData", []);
  GetWorldPoint(localPoint) => jsvalue.callMethod(r"GetWorldPoint", [localPoint]);
  GetWorldVector(localVector) => jsvalue.callMethod(r"GetWorldVector", [localVector]);
  GetLocalPoint(worldPoint) => jsvalue.callMethod(r"GetLocalPoint", [worldPoint]);
  GetLocalVector(worldVector) => jsvalue.callMethod(r"GetLocalVector", [worldVector]);
  GetLinearVelocityFromWorldPoint(worldPoint) => jsvalue.callMethod(r"GetLinearVelocityFromWorldPoint", [worldPoint]);
  GetLinearVelocityFromLocalPoint(localPoint) => jsvalue.callMethod(r"GetLinearVelocityFromLocalPoint", [localPoint]);
  GetLinearDamping() => jsvalue.callMethod(r"GetLinearDamping", []);
  SetLinearDamping(linearDamping) => jsvalue.callMethod(r"SetLinearDamping", [linearDamping]);
  GetAngularDamping() => jsvalue.callMethod(r"GetAngularDamping", []);
  SetAngularDamping(angularDamping) => jsvalue.callMethod(r"SetAngularDamping", [angularDamping]);
  SetType(type) => jsvalue.callMethod(r"SetType", [type]);
  GetType() => jsvalue.callMethod(r"GetType", []);
  SetBullet(flag) => jsvalue.callMethod(r"SetBullet", [flag]);
  IsBullet() => jsvalue.callMethod(r"IsBullet", []);
  SetSleepingAllowed(flag) => jsvalue.callMethod(r"SetSleepingAllowed", [flag]);
  SetAwake(flag) => jsvalue.callMethod(r"SetAwake", [flag]);
  IsAwake() => jsvalue.callMethod(r"IsAwake", []);
  SetFixedRotation(fixed) => jsvalue.callMethod(r"SetFixedRotation", [fixed]);
  IsFixedRotation() => jsvalue.callMethod(r"IsFixedRotation", []);
  SetActive(flag) => jsvalue.callMethod(r"SetActive", [flag]);
  IsActive() => jsvalue.callMethod(r"IsActive", []);
  IsSleepingAllowed() => jsvalue.callMethod(r"IsSleepingAllowed", []);
  GetFixtureList() => jsvalue.callMethod(r"GetFixtureList", []);
  GetJointList() => jsvalue.callMethod(r"GetJointList", []);
  GetControllerList() => jsvalue.callMethod(r"GetControllerList", []);
  GetContactList() => jsvalue.callMethod(r"GetContactList", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetUserData() => jsvalue.callMethod(r"GetUserData", []);
  SetUserData(data) => jsvalue.callMethod(r"SetUserData", [data]);
  GetWorld() => jsvalue.callMethod(r"GetWorld", []);
  b2Body(bd, world) => jsvalue.callMethod(r"b2Body", [bd, world]);
  SynchronizeFixtures() => jsvalue.callMethod(r"SynchronizeFixtures", []);
  SynchronizeTransform() => jsvalue.callMethod(r"SynchronizeTransform", []);
  ShouldCollide(other) => jsvalue.callMethod(r"ShouldCollide", [other]);
  Advance(t) => jsvalue.callMethod(r"Advance", [t]);
  static b2Body() => _constructor.callMethod(r"b2Body", []);
}

class _b2BodyDef implements b2BodyDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2BodyDef"];
  _b2BodyDef() : jsvalue = new JsObject(_constructor, []);

  b2BodyDef() => jsvalue.callMethod(r"b2BodyDef", []);
  static b2BodyDef() => _constructor.callMethod(r"b2BodyDef", []);
}

class _b2ContactManager implements b2ContactManager {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactManager"];
  _b2ContactManager() : jsvalue = new JsObject(_constructor, []);

  b2ContactManager() => jsvalue.callMethod(r"b2ContactManager", []);
  AddPair(proxyUserDataA, proxyUserDataB) => jsvalue.callMethod(r"AddPair", [proxyUserDataA, proxyUserDataB]);
  FindNewContacts() => jsvalue.callMethod(r"FindNewContacts", []);
  Destroy(c) => jsvalue.callMethod(r"Destroy", [c]);
  Collide() => jsvalue.callMethod(r"Collide", []);
  static b2ContactManager() => _constructor.callMethod(r"b2ContactManager", []);
}

class _b2DebugDraw implements b2DebugDraw {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2DebugDraw"];
  _b2DebugDraw() : jsvalue = new JsObject(_constructor, []);

  b2DebugDraw() => jsvalue.callMethod(r"b2DebugDraw", []);
  SetFlags(flags) => jsvalue.callMethod(r"SetFlags", [flags]);
  GetFlags() => jsvalue.callMethod(r"GetFlags", []);
  AppendFlags(flags) => jsvalue.callMethod(r"AppendFlags", [flags]);
  ClearFlags(flags) => jsvalue.callMethod(r"ClearFlags", [flags]);
  SetSprite(sprite) => jsvalue.callMethod(r"SetSprite", [sprite]);
  GetSprite() => jsvalue.callMethod(r"GetSprite", []);
  SetDrawScale(drawScale) => jsvalue.callMethod(r"SetDrawScale", [drawScale]);
  GetDrawScale() => jsvalue.callMethod(r"GetDrawScale", []);
  SetLineThickness(lineThickness) => jsvalue.callMethod(r"SetLineThickness", [lineThickness]);
  GetLineThickness() => jsvalue.callMethod(r"GetLineThickness", []);
  SetAlpha(alpha) => jsvalue.callMethod(r"SetAlpha", [alpha]);
  GetAlpha() => jsvalue.callMethod(r"GetAlpha", []);
  SetFillAlpha(alpha) => jsvalue.callMethod(r"SetFillAlpha", [alpha]);
  GetFillAlpha() => jsvalue.callMethod(r"GetFillAlpha", []);
  SetXFormScale(xformScale) => jsvalue.callMethod(r"SetXFormScale", [xformScale]);
  GetXFormScale() => jsvalue.callMethod(r"GetXFormScale", []);
  DrawPolygon(vertices, vertexCount, color) => jsvalue.callMethod(r"DrawPolygon", [vertices, vertexCount, color]);
  DrawSolidPolygon(vertices, vertexCount, color) => jsvalue.callMethod(r"DrawSolidPolygon", [vertices, vertexCount, color]);
  DrawCircle(center, radius, color) => jsvalue.callMethod(r"DrawCircle", [center, radius, color]);
  DrawSolidCircle(center, radius, axis, color) => jsvalue.callMethod(r"DrawSolidCircle", [center, radius, axis, color]);
  DrawSegment(p1, p2, color) => jsvalue.callMethod(r"DrawSegment", [p1, p2, color]);
  DrawTransform(xf) => jsvalue.callMethod(r"DrawTransform", [xf]);
  static b2DebugDraw() => _constructor.callMethod(r"b2DebugDraw", []);
}

class _b2DestructionListener implements b2DestructionListener {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2DestructionListener"];
  _b2DestructionListener() : jsvalue = new JsObject(_constructor, []);

  SayGoodbyeJoint(joint) => jsvalue.callMethod(r"SayGoodbyeJoint", [joint]);
  SayGoodbyeFixture(fixture) => jsvalue.callMethod(r"SayGoodbyeFixture", [fixture]);
  static b2DestructionListener() => _constructor.callMethod(r"b2DestructionListener", []);
}

class _b2FilterData implements b2FilterData {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2FilterData"];
  _b2FilterData() : jsvalue = new JsObject(_constructor, []);

  Copy() => jsvalue.callMethod(r"Copy", []);
  static b2FilterData() => _constructor.callMethod(r"b2FilterData", []);
}

class _b2Fixture implements b2Fixture {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2Fixture"];
  _b2Fixture() : jsvalue = new JsObject(_constructor, []);

  GetType() => jsvalue.callMethod(r"GetType", []);
  GetShape() => jsvalue.callMethod(r"GetShape", []);
  SetSensor(sensor) => jsvalue.callMethod(r"SetSensor", [sensor]);
  IsSensor() => jsvalue.callMethod(r"IsSensor", []);
  SetFilterData(filter) => jsvalue.callMethod(r"SetFilterData", [filter]);
  GetFilterData() => jsvalue.callMethod(r"GetFilterData", []);
  GetBody() => jsvalue.callMethod(r"GetBody", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetUserData() => jsvalue.callMethod(r"GetUserData", []);
  SetUserData(data) => jsvalue.callMethod(r"SetUserData", [data]);
  TestPoint(p) => jsvalue.callMethod(r"TestPoint", [p]);
  RayCast(output, input) => jsvalue.callMethod(r"RayCast", [output, input]);
  GetMassData(massData) => jsvalue.callMethod(r"GetMassData", [massData]);
  SetDensity(density) => jsvalue.callMethod(r"SetDensity", [density]);
  GetDensity() => jsvalue.callMethod(r"GetDensity", []);
  GetFriction() => jsvalue.callMethod(r"GetFriction", []);
  SetFriction(friction) => jsvalue.callMethod(r"SetFriction", [friction]);
  GetRestitution() => jsvalue.callMethod(r"GetRestitution", []);
  SetRestitution(restitution) => jsvalue.callMethod(r"SetRestitution", [restitution]);
  GetAABB() => jsvalue.callMethod(r"GetAABB", []);
  b2Fixture() => jsvalue.callMethod(r"b2Fixture", []);
  Create(body, xf, def) => jsvalue.callMethod(r"Create", [body, xf, def]);
  Destroy() => jsvalue.callMethod(r"Destroy", []);
  CreateProxy(broadPhase, xf) => jsvalue.callMethod(r"CreateProxy", [broadPhase, xf]);
  DestroyProxy(broadPhase) => jsvalue.callMethod(r"DestroyProxy", [broadPhase]);
  Synchronize(broadPhase, transform1, transform2) => jsvalue.callMethod(r"Synchronize", [broadPhase, transform1, transform2]);
  static b2Fixture() => _constructor.callMethod(r"b2Fixture", []);
}

class _b2FixtureDef implements b2FixtureDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2FixtureDef"];
  _b2FixtureDef() : jsvalue = new JsObject(_constructor, []);

  b2FixtureDef() => jsvalue.callMethod(r"b2FixtureDef", []);
  static b2FixtureDef() => _constructor.callMethod(r"b2FixtureDef", []);
}

class _b2Island implements b2Island {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2Island"];
  _b2Island() : jsvalue = new JsObject(_constructor, []);

  b2Island() => jsvalue.callMethod(r"b2Island", []);
  Initialize(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) => jsvalue.callMethod(r"Initialize", [bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver]);
  Clear() => jsvalue.callMethod(r"Clear", []);
  Solve(step, gravity, allowSleep) => jsvalue.callMethod(r"Solve", [step, gravity, allowSleep]);
  SolveTOI(subStep) => jsvalue.callMethod(r"SolveTOI", [subStep]);
  Report(constraints) => jsvalue.callMethod(r"Report", [constraints]);
  AddBody(body) => jsvalue.callMethod(r"AddBody", [body]);
  AddContact(contact) => jsvalue.callMethod(r"AddContact", [contact]);
  AddJoint(joint) => jsvalue.callMethod(r"AddJoint", [joint]);
  static b2Island() => _constructor.callMethod(r"b2Island", []);
}

class _b2World implements b2World {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2World"];
  _b2World(gravity, doSleep) : jsvalue = new JsObject(_constructor, [gravity, doSleep]);

  b2World(gravity, doSleep) => jsvalue.callMethod(r"b2World", [gravity, doSleep]);
  SetDestructionListener(listener) => jsvalue.callMethod(r"SetDestructionListener", [listener]);
  SetContactFilter(filter) => jsvalue.callMethod(r"SetContactFilter", [filter]);
  SetContactListener(listener) => jsvalue.callMethod(r"SetContactListener", [listener]);
  SetDebugDraw(debugDraw) => jsvalue.callMethod(r"SetDebugDraw", [debugDraw]);
  SetBroadPhase(broadPhase) => jsvalue.callMethod(r"SetBroadPhase", [broadPhase]);
  Validate() => jsvalue.callMethod(r"Validate", []);
  GetProxyCount() => jsvalue.callMethod(r"GetProxyCount", []);
  CreateBody(def) => jsvalue.callMethod(r"CreateBody", [def]);
  DestroyBody(b) => jsvalue.callMethod(r"DestroyBody", [b]);
  CreateJoint(def) => jsvalue.callMethod(r"CreateJoint", [def]);
  DestroyJoint(j) => jsvalue.callMethod(r"DestroyJoint", [j]);
  AddController(c) => jsvalue.callMethod(r"AddController", [c]);
  RemoveController(c) => jsvalue.callMethod(r"RemoveController", [c]);
  CreateController(controller) => jsvalue.callMethod(r"CreateController", [controller]);
  DestroyController(controller) => jsvalue.callMethod(r"DestroyController", [controller]);
  SetWarmStarting(flag) => jsvalue.callMethod(r"SetWarmStarting", [flag]);
  SetContinuousPhysics(flag) => jsvalue.callMethod(r"SetContinuousPhysics", [flag]);
  GetBodyCount() => jsvalue.callMethod(r"GetBodyCount", []);
  GetJointCount() => jsvalue.callMethod(r"GetJointCount", []);
  GetContactCount() => jsvalue.callMethod(r"GetContactCount", []);
  SetGravity(gravity) => jsvalue.callMethod(r"SetGravity", [gravity]);
  GetGravity() => jsvalue.callMethod(r"GetGravity", []);
  GetGroundBody() => jsvalue.callMethod(r"GetGroundBody", []);
  Step(dt, velocityIterations, positionIterations) => jsvalue.callMethod(r"Step", [dt, velocityIterations, positionIterations]);
  ClearForces() => jsvalue.callMethod(r"ClearForces", []);
  DrawDebugData() => jsvalue.callMethod(r"DrawDebugData", []);
  QueryAABB(callback, aabb) => jsvalue.callMethod(r"QueryAABB", [callback, aabb]);
  QueryShape(callback, shape, transform) => jsvalue.callMethod(r"QueryShape", [callback, shape, transform]);
  QueryPoint(callback, p) => jsvalue.callMethod(r"QueryPoint", [callback, p]);
  RayCast(callback, point1, point2) => jsvalue.callMethod(r"RayCast", [callback, point1, point2]);
  RayCastOne(point1, point2) => jsvalue.callMethod(r"RayCastOne", [point1, point2]);
  RayCastAll(point1, point2) => jsvalue.callMethod(r"RayCastAll", [point1, point2]);
  GetBodyList() => jsvalue.callMethod(r"GetBodyList", []);
  GetJointList() => jsvalue.callMethod(r"GetJointList", []);
  GetContactList() => jsvalue.callMethod(r"GetContactList", []);
  IsLocked() => jsvalue.callMethod(r"IsLocked", []);
  Solve(step) => jsvalue.callMethod(r"Solve", [step]);
  SolveTOI(step) => jsvalue.callMethod(r"SolveTOI", [step]);
  DrawJoint(joint) => jsvalue.callMethod(r"DrawJoint", [joint]);
  DrawShape(shape, xf, color) => jsvalue.callMethod(r"DrawShape", [shape, xf, color]);
  static b2World() => _constructor.callMethod(r"b2World", []);
}

class _b2AABB implements b2AABB {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2AABB"];
  _b2AABB() : jsvalue = new JsObject(_constructor, []);

  IsValid() => jsvalue.callMethod(r"IsValid", []);
  GetCenter() => jsvalue.callMethod(r"GetCenter", []);
  GetExtents() => jsvalue.callMethod(r"GetExtents", []);
  Contains(aabb) => jsvalue.callMethod(r"Contains", [aabb]);
  RayCast(output, input) => jsvalue.callMethod(r"RayCast", [output, input]);
  TestOverlap(other) => jsvalue.callMethod(r"TestOverlap", [other]);
  Combine(aabb1, aabb2) => jsvalue.callMethod(r"Combine", [aabb1, aabb2]);
  static b2AABB() => _constructor.callMethod(r"b2AABB", []);
  static Combine(aabb1, aabb2) => _constructor.callMethod(r"Combine", [aabb1, aabb2]);
}

class _b2Bound implements b2Bound {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Bound"];
  _b2Bound() : jsvalue = new JsObject(_constructor, []);

  IsLower() => jsvalue.callMethod(r"IsLower", []);
  IsUpper() => jsvalue.callMethod(r"IsUpper", []);
  Swap(b) => jsvalue.callMethod(r"Swap", [b]);
  static b2Bound() => _constructor.callMethod(r"b2Bound", []);
}

class _b2BoundValues implements b2BoundValues {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2BoundValues"];
  _b2BoundValues() : jsvalue = new JsObject(_constructor, []);

  b2BoundValues() => jsvalue.callMethod(r"b2BoundValues", []);
  static b2BoundValues() => _constructor.callMethod(r"b2BoundValues", []);
}

class _b2DynamicTree implements b2DynamicTree {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTree"];
  _b2DynamicTree() : jsvalue = new JsObject(_constructor, []);

  b2DynamicTree() => jsvalue.callMethod(r"b2DynamicTree", []);
  CreateProxy(aabb, userData) => jsvalue.callMethod(r"CreateProxy", [aabb, userData]);
  DestroyProxy(proxy) => jsvalue.callMethod(r"DestroyProxy", [proxy]);
  MoveProxy(proxy, aabb, displacement) => jsvalue.callMethod(r"MoveProxy", [proxy, aabb, displacement]);
  Rebalance(iterations) => jsvalue.callMethod(r"Rebalance", [iterations]);
  GetFatAABB(proxy) => jsvalue.callMethod(r"GetFatAABB", [proxy]);
  GetUserData(proxy) => jsvalue.callMethod(r"GetUserData", [proxy]);
  Query(callback, aabb) => jsvalue.callMethod(r"Query", [callback, aabb]);
  RayCast(callback, input) => jsvalue.callMethod(r"RayCast", [callback, input]);
  AllocateNode() => jsvalue.callMethod(r"AllocateNode", []);
  FreeNode(node) => jsvalue.callMethod(r"FreeNode", [node]);
  InsertLeaf(leaf) => jsvalue.callMethod(r"InsertLeaf", [leaf]);
  RemoveLeaf(leaf) => jsvalue.callMethod(r"RemoveLeaf", [leaf]);
  static b2DynamicTree() => _constructor.callMethod(r"b2DynamicTree", []);
}

class _b2DynamicTreeBroadPhase implements b2DynamicTreeBroadPhase {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTreeBroadPhase"];
  _b2DynamicTreeBroadPhase() : jsvalue = new JsObject(_constructor, []);

  CreateProxy(aabb, userData) => jsvalue.callMethod(r"CreateProxy", [aabb, userData]);
  DestroyProxy(proxy) => jsvalue.callMethod(r"DestroyProxy", [proxy]);
  MoveProxy(proxy, aabb, displacement) => jsvalue.callMethod(r"MoveProxy", [proxy, aabb, displacement]);
  TestOverlap(proxyA, proxyB) => jsvalue.callMethod(r"TestOverlap", [proxyA, proxyB]);
  GetUserData(proxy) => jsvalue.callMethod(r"GetUserData", [proxy]);
  GetFatAABB(proxy) => jsvalue.callMethod(r"GetFatAABB", [proxy]);
  GetProxyCount() => jsvalue.callMethod(r"GetProxyCount", []);
  UpdatePairs(callback) => jsvalue.callMethod(r"UpdatePairs", [callback]);
  Query(callback, aabb) => jsvalue.callMethod(r"Query", [callback, aabb]);
  RayCast(callback, input) => jsvalue.callMethod(r"RayCast", [callback, input]);
  Validate() => jsvalue.callMethod(r"Validate", []);
  Rebalance(iterations) => jsvalue.callMethod(r"Rebalance", [iterations]);
  BufferMove(proxy) => jsvalue.callMethod(r"BufferMove", [proxy]);
  UnBufferMove(proxy) => jsvalue.callMethod(r"UnBufferMove", [proxy]);
  ComparePairs(pair1, pair2) => jsvalue.callMethod(r"ComparePairs", [pair1, pair2]);
  static b2DynamicTreeBroadPhase() => _constructor.callMethod(r"b2DynamicTreeBroadPhase", []);
}

class _b2DynamicTreeNode implements b2DynamicTreeNode {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTreeNode"];
  _b2DynamicTreeNode() : jsvalue = new JsObject(_constructor, []);

  IsLeaf() => jsvalue.callMethod(r"IsLeaf", []);
  static b2DynamicTreeNode() => _constructor.callMethod(r"b2DynamicTreeNode", []);
}

class _b2Manifold implements b2Manifold {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Manifold"];
  _b2Manifold() : jsvalue = new JsObject(_constructor, []);

  b2Manifold() => jsvalue.callMethod(r"b2Manifold", []);
  Reset() => jsvalue.callMethod(r"Reset", []);
  Set(m) => jsvalue.callMethod(r"Set", [m]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  static b2Manifold() => _constructor.callMethod(r"b2Manifold", []);
}

class _b2ManifoldPoint implements b2ManifoldPoint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2ManifoldPoint"];
  _b2ManifoldPoint() : jsvalue = new JsObject(_constructor, []);

  b2ManifoldPoint() => jsvalue.callMethod(r"b2ManifoldPoint", []);
  Reset() => jsvalue.callMethod(r"Reset", []);
  Set(m) => jsvalue.callMethod(r"Set", [m]);
  static b2ManifoldPoint() => _constructor.callMethod(r"b2ManifoldPoint", []);
}

class _b2Point implements b2Point {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Point"];
  _b2Point() : jsvalue = new JsObject(_constructor, []);

  Support(xf, vX, vY) => jsvalue.callMethod(r"Support", [xf, vX, vY]);
  GetFirstVertex(xf) => jsvalue.callMethod(r"GetFirstVertex", [xf]);
  static b2Point() => _constructor.callMethod(r"b2Point", []);
}

class _b2RayCastInput implements b2RayCastInput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2RayCastInput"];
  _b2RayCastInput(p1, p2, maxFraction) : jsvalue = new JsObject(_constructor, [p1, p2, maxFraction]);

  b2RayCastInput(p1, p2, maxFraction) => jsvalue.callMethod(r"b2RayCastInput", [p1, p2, maxFraction]);
  static b2RayCastInput() => _constructor.callMethod(r"b2RayCastInput", []);
}

class _b2Segment implements b2Segment {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Segment"];
  _b2Segment() : jsvalue = new JsObject(_constructor, []);

  TestSegment(lambda, normal, segment, maxLambda) => jsvalue.callMethod(r"TestSegment", [lambda, normal, segment, maxLambda]);
  Extend(aabb) => jsvalue.callMethod(r"Extend", [aabb]);
  ExtendForward(aabb) => jsvalue.callMethod(r"ExtendForward", [aabb]);
  ExtendBackward(aabb) => jsvalue.callMethod(r"ExtendBackward", [aabb]);
  static b2Segment() => _constructor.callMethod(r"b2Segment", []);
}

class _b2CircleShape extends _b2Shape implements b2CircleShape {
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2CircleShape"];
  _b2CircleShape(radius) : super.withValue(new JsObject(_constructor, [radius]));

  Copy() => jsvalue.callMethod(r"Copy", []);
  Set(other) => jsvalue.callMethod(r"Set", [other]);
  TestPoint(transform, p) => jsvalue.callMethod(r"TestPoint", [transform, p]);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [output, input, transform]);
  ComputeAABB(aabb, transform) => jsvalue.callMethod(r"ComputeAABB", [aabb, transform]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [massData, density]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [normal, offset, xf, c]);
  GetLocalPosition() => jsvalue.callMethod(r"GetLocalPosition", []);
  SetLocalPosition(position) => jsvalue.callMethod(r"SetLocalPosition", [position]);
  GetRadius() => jsvalue.callMethod(r"GetRadius", []);
  SetRadius(radius) => jsvalue.callMethod(r"SetRadius", [radius]);
  b2CircleShape(radius) => jsvalue.callMethod(r"b2CircleShape", [radius]);
  static b2CircleShape() => _constructor.callMethod(r"b2CircleShape", []);
}

class _b2EdgeChainDef implements b2EdgeChainDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2EdgeChainDef"];
  _b2EdgeChainDef() : jsvalue = new JsObject(_constructor, []);

  b2EdgeChainDef() => jsvalue.callMethod(r"b2EdgeChainDef", []);
  static b2EdgeChainDef() => _constructor.callMethod(r"b2EdgeChainDef", []);
}

class _b2EdgeShape extends _b2Shape implements b2EdgeShape {
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2EdgeShape"];
  _b2EdgeShape(v1, v2) : super.withValue(new JsObject(_constructor, [v1, v2]));

  TestPoint(transform, p) => jsvalue.callMethod(r"TestPoint", [transform, p]);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [output, input, transform]);
  ComputeAABB(aabb, transform) => jsvalue.callMethod(r"ComputeAABB", [aabb, transform]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [massData, density]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [normal, offset, xf, c]);
  GetLength() => jsvalue.callMethod(r"GetLength", []);
  GetVertex1() => jsvalue.callMethod(r"GetVertex1", []);
  GetVertex2() => jsvalue.callMethod(r"GetVertex2", []);
  GetCoreVertex1() => jsvalue.callMethod(r"GetCoreVertex1", []);
  GetCoreVertex2() => jsvalue.callMethod(r"GetCoreVertex2", []);
  GetNormalVector() => jsvalue.callMethod(r"GetNormalVector", []);
  GetDirectionVector() => jsvalue.callMethod(r"GetDirectionVector", []);
  GetCorner1Vector() => jsvalue.callMethod(r"GetCorner1Vector", []);
  GetCorner2Vector() => jsvalue.callMethod(r"GetCorner2Vector", []);
  Corner1IsConvex() => jsvalue.callMethod(r"Corner1IsConvex", []);
  Corner2IsConvex() => jsvalue.callMethod(r"Corner2IsConvex", []);
  GetFirstVertex(xf) => jsvalue.callMethod(r"GetFirstVertex", [xf]);
  GetNextEdge() => jsvalue.callMethod(r"GetNextEdge", []);
  GetPrevEdge() => jsvalue.callMethod(r"GetPrevEdge", []);
  Support(xf, dX, dY) => jsvalue.callMethod(r"Support", [xf, dX, dY]);
  b2EdgeShape(v1, v2) => jsvalue.callMethod(r"b2EdgeShape", [v1, v2]);
  SetPrevEdge(edge, core, cornerDir, convex) => jsvalue.callMethod(r"SetPrevEdge", [edge, core, cornerDir, convex]);
  SetNextEdge(edge, core, cornerDir, convex) => jsvalue.callMethod(r"SetNextEdge", [edge, core, cornerDir, convex]);
  static b2EdgeShape() => _constructor.callMethod(r"b2EdgeShape", []);
}

class _b2PolygonShape extends _b2Shape implements b2PolygonShape {
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2PolygonShape"];
  _b2PolygonShape() : super.withValue(new JsObject(_constructor, []));

  Copy() => jsvalue.callMethod(r"Copy", []);
  Set(other) => jsvalue.callMethod(r"Set", [other]);
  SetAsArray(vertices, vertexCount) => jsvalue.callMethod(r"SetAsArray", [vertices, vertexCount]);
  SetAsVector(vertices, vertexCount) => jsvalue.callMethod(r"SetAsVector", [vertices, vertexCount]);
  SetAsBox(hx, hy) => jsvalue.callMethod(r"SetAsBox", [hx, hy]);
  SetAsOrientedBox(hx, hy, center, angle) => jsvalue.callMethod(r"SetAsOrientedBox", [hx, hy, center, angle]);
  SetAsEdge(v1, v2) => jsvalue.callMethod(r"SetAsEdge", [v1, v2]);
  TestPoint(xf, p) => jsvalue.callMethod(r"TestPoint", [xf, p]);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [output, input, transform]);
  ComputeAABB(aabb, xf) => jsvalue.callMethod(r"ComputeAABB", [aabb, xf]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [massData, density]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [normal, offset, xf, c]);
  GetVertexCount() => jsvalue.callMethod(r"GetVertexCount", []);
  GetVertices() => jsvalue.callMethod(r"GetVertices", []);
  GetNormals() => jsvalue.callMethod(r"GetNormals", []);
  GetSupport(d) => jsvalue.callMethod(r"GetSupport", [d]);
  GetSupportVertex(d) => jsvalue.callMethod(r"GetSupportVertex", [d]);
  Validate() => jsvalue.callMethod(r"Validate", []);
  b2PolygonShape() => jsvalue.callMethod(r"b2PolygonShape", []);
  Reserve(count) => jsvalue.callMethod(r"Reserve", [count]);
  static b2PolygonShape() => _constructor.callMethod(r"b2PolygonShape", []);
  static AsArray(vertices, vertexCount) => _constructor.callMethod(r"AsArray", [vertices, vertexCount]);
  static AsVector(vertices, vertexCount) => _constructor.callMethod(r"AsVector", [vertices, vertexCount]);
  static AsBox(hx, hy) => _constructor.callMethod(r"AsBox", [hx, hy]);
  static AsOrientedBox(hx, hy, center, angle) => _constructor.callMethod(r"AsOrientedBox", [hx, hy, center, angle]);
  static AsEdge(v1, v2) => _constructor.callMethod(r"AsEdge", [v1, v2]);
  static ComputeCentroid(vs, count) => _constructor.callMethod(r"ComputeCentroid", [vs, count]);
  static ComputeOBB(obb, vs, count) => _constructor.callMethod(r"ComputeOBB", [obb, vs, count]);
}

class _b2CircleContact extends _b2Contact implements b2CircleContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2CircleContact"];
  _b2CircleContact() : super.withValue(new JsObject(_constructor, []));

  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [fixtureA, fixtureB]);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  static b2CircleContact() => _constructor.callMethod(r"b2CircleContact", []);
  static Create(allocator) => _constructor.callMethod(r"Create", [allocator]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [contact, allocator]);
}

class _b2ContactConstraint implements b2ContactConstraint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactConstraint"];
  _b2ContactConstraint() : jsvalue = new JsObject(_constructor, []);

  b2ContactConstraint() => jsvalue.callMethod(r"b2ContactConstraint", []);
  static b2ContactConstraint() => _constructor.callMethod(r"b2ContactConstraint", []);
}

class _b2ContactFactory implements b2ContactFactory {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactFactory"];
  _b2ContactFactory(allocator) : jsvalue = new JsObject(_constructor, [allocator]);

  b2ContactFactory(allocator) => jsvalue.callMethod(r"b2ContactFactory", [allocator]);
  AddType(createFcn, destroyFcn, type1, type2) => jsvalue.callMethod(r"AddType", [createFcn, destroyFcn, type1, type2]);
  InitializeRegisters() => jsvalue.callMethod(r"InitializeRegisters", []);
  Create(fixtureA, fixtureB) => jsvalue.callMethod(r"Create", [fixtureA, fixtureB]);
  Destroy(contact) => jsvalue.callMethod(r"Destroy", [contact]);
  static b2ContactFactory() => _constructor.callMethod(r"b2ContactFactory", []);
}

class _b2ContactSolver implements b2ContactSolver {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactSolver"];
  _b2ContactSolver() : jsvalue = new JsObject(_constructor, []);

  b2ContactSolver() => jsvalue.callMethod(r"b2ContactSolver", []);
  Initialize(step, contacts, contactCount, allocator) => jsvalue.callMethod(r"Initialize", [step, contacts, contactCount, allocator]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [step]);
  SolveVelocityConstraints() => jsvalue.callMethod(r"SolveVelocityConstraints", []);
  FinalizeVelocityConstraints() => jsvalue.callMethod(r"FinalizeVelocityConstraints", []);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [baumgarte]);
  static b2ContactSolver() => _constructor.callMethod(r"b2ContactSolver", []);
}

class _b2EdgeAndCircleContact extends _b2Contact implements b2EdgeAndCircleContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2EdgeAndCircleContact"];
  _b2EdgeAndCircleContact() : super.withValue(new JsObject(_constructor, []));

  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [fixtureA, fixtureB]);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  b2CollideEdgeAndCircle(manifold, edge, xf1, circle, xf2) => jsvalue.callMethod(r"b2CollideEdgeAndCircle", [manifold, edge, xf1, circle, xf2]);
  static b2EdgeAndCircleContact() => _constructor.callMethod(r"b2EdgeAndCircleContact", []);
  static Create(allocator) => _constructor.callMethod(r"Create", [allocator]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [contact, allocator]);
}

class _b2NullContact extends _b2Contact implements b2NullContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2NullContact"];
  _b2NullContact() : super.withValue(new JsObject(_constructor, []));

  b2NullContact() => jsvalue.callMethod(r"b2NullContact", []);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  static b2NullContact() => _constructor.callMethod(r"b2NullContact", []);
}

class _b2PolyAndCircleContact extends _b2Contact implements b2PolyAndCircleContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PolyAndCircleContact"];
  _b2PolyAndCircleContact() : super.withValue(new JsObject(_constructor, []));

  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [fixtureA, fixtureB]);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  static b2PolyAndCircleContact() => _constructor.callMethod(r"b2PolyAndCircleContact", []);
  static Create(allocator) => _constructor.callMethod(r"Create", [allocator]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [contact, allocator]);
}

class _b2PolyAndEdgeContact extends _b2Contact implements b2PolyAndEdgeContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PolyAndEdgeContact"];
  _b2PolyAndEdgeContact() : super.withValue(new JsObject(_constructor, []));

  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [fixtureA, fixtureB]);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  b2CollidePolyAndEdge(manifold, polygon, xf1, edge, xf2) => jsvalue.callMethod(r"b2CollidePolyAndEdge", [manifold, polygon, xf1, edge, xf2]);
  static b2PolyAndEdgeContact() => _constructor.callMethod(r"b2PolyAndEdgeContact", []);
  static Create(allocator) => _constructor.callMethod(r"Create", [allocator]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [contact, allocator]);
}

class _b2PolygonContact extends _b2Contact implements b2PolygonContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PolygonContact"];
  _b2PolygonContact() : super.withValue(new JsObject(_constructor, []));

  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [fixtureA, fixtureB]);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  static b2PolygonContact() => _constructor.callMethod(r"b2PolygonContact", []);
  static Create(allocator) => _constructor.callMethod(r"Create", [allocator]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [contact, allocator]);
}

class _b2BuoyancyController extends _b2Controller implements b2BuoyancyController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2BuoyancyController"];
  _b2BuoyancyController() : super.withValue(new JsObject(_constructor, []));

  Step(step) => jsvalue.callMethod(r"Step", [step]);
  Draw(debugDraw) => jsvalue.callMethod(r"Draw", [debugDraw]);
  static b2BuoyancyController() => _constructor.callMethod(r"b2BuoyancyController", []);
}

class _b2ConstantAccelController extends _b2Controller implements b2ConstantAccelController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2ConstantAccelController"];
  _b2ConstantAccelController() : super.withValue(new JsObject(_constructor, []));

  Step(step) => jsvalue.callMethod(r"Step", [step]);
  static b2ConstantAccelController() => _constructor.callMethod(r"b2ConstantAccelController", []);
}

class _b2ConstantForceController extends _b2Controller implements b2ConstantForceController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2ConstantForceController"];
  _b2ConstantForceController() : super.withValue(new JsObject(_constructor, []));

  Step(step) => jsvalue.callMethod(r"Step", [step]);
  static b2ConstantForceController() => _constructor.callMethod(r"b2ConstantForceController", []);
}

class _b2GravityController extends _b2Controller implements b2GravityController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2GravityController"];
  _b2GravityController() : super.withValue(new JsObject(_constructor, []));

  Step(step) => jsvalue.callMethod(r"Step", [step]);
  static b2GravityController() => _constructor.callMethod(r"b2GravityController", []);
}

class _b2TensorDampingController extends _b2Controller implements b2TensorDampingController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2TensorDampingController"];
  _b2TensorDampingController() : super.withValue(new JsObject(_constructor, []));

  SetAxisAligned(xDamping, yDamping) => jsvalue.callMethod(r"SetAxisAligned", [xDamping, yDamping]);
  Step(step) => jsvalue.callMethod(r"Step", [step]);
  static b2TensorDampingController() => _constructor.callMethod(r"b2TensorDampingController", []);
}

class _b2ControllerEdge implements b2ControllerEdge {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2ControllerEdge"];
  _b2ControllerEdge() : jsvalue = new JsObject(_constructor, []);

  static b2ControllerEdge() => _constructor.callMethod(r"b2ControllerEdge", []);
}

class _Vector_a2j_Number implements Vector_a2j_Number {
  var jsvalue;
  static JsObject _constructor = context["Vector_a2j_Number"];
  _Vector_a2j_Number(length) : jsvalue = new JsObject(_constructor, [length]);

}

class _b2ContactRegister implements b2ContactRegister {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactRegister"];
  _b2ContactRegister() : jsvalue = new JsObject(_constructor, []);

  static b2ContactRegister() => _constructor.callMethod(r"b2ContactRegister", []);
}

class _b2ContactConstraintPoint implements b2ContactConstraintPoint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactConstraintPoint"];
  _b2ContactConstraintPoint() : jsvalue = new JsObject(_constructor, []);

  static b2ContactConstraintPoint() => _constructor.callMethod(r"b2ContactConstraintPoint", []);
}

class _b2ContactEdge implements b2ContactEdge {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactEdge"];
  _b2ContactEdge() : jsvalue = new JsObject(_constructor, []);

  static b2ContactEdge() => _constructor.callMethod(r"b2ContactEdge", []);
}

class _b2MassData implements b2MassData {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2MassData"];
  _b2MassData() : jsvalue = new JsObject(_constructor, []);

  static b2MassData() => _constructor.callMethod(r"b2MassData", []);
}

class _b2DynamicTreePair implements b2DynamicTreePair {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTreePair"];
  _b2DynamicTreePair() : jsvalue = new JsObject(_constructor, []);

  static b2DynamicTreePair() => _constructor.callMethod(r"b2DynamicTreePair", []);
}

class _b2RayCastOutput implements b2RayCastOutput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2RayCastOutput"];
  _b2RayCastOutput() : jsvalue = new JsObject(_constructor, []);

  static b2RayCastOutput() => _constructor.callMethod(r"b2RayCastOutput", []);
}

class _b2JointEdge implements b2JointEdge {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2JointEdge"];
  _b2JointEdge() : jsvalue = new JsObject(_constructor, []);

  static b2JointEdge() => _constructor.callMethod(r"b2JointEdge", []);
}

