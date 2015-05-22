library box2dweb;

import 'dart:js';

abstract class b2TimeStep {
  factory b2TimeStep() = _b2TimeStep;

  Set(step);

  static b2TimeStep() => _b2TimeStep.b2TimeStep();
}

abstract class b2Transform {
  factory b2Transform(pos, r) = _b2Transform;

  GetAngle();
  Initialize(pos, r);
  Set(x);
  SetIdentity();
  b2Transform(pos, r);

  static b2Transform() => _b2Transform.b2Transform();
}

abstract class b2Sweep {
  factory b2Sweep() = _b2Sweep;

  Advance(t);
  Copy();
  GetTransform(xf, alpha);
  Set(other);

  static b2Sweep() => _b2Sweep.b2Sweep();
}

abstract class b2Color {
  factory b2Color(rr, gg, bb) = _b2Color;

  Set(rr, gg, bb);
  b2Color(rr, gg, bb);

  static b2Color() => _b2Color.b2Color();
}

abstract class b2Controller {
  factory b2Controller() = _b2Controller;

  AddBody(body);
  Clear();
  Draw(debugDraw);
  GetBodyList();
  GetNext();
  GetWorld();
  RemoveBody(body);
  Step(step);

  static b2Controller() => _b2Controller.b2Controller();
}

abstract class b2Vec2 {
  factory b2Vec2(x_, y_) = _b2Vec2;

  Abs();
  Add(v);
  Copy();
  CrossFV(s);
  CrossVF(s);
  GetNegative();
  IsValid();
  Length();
  LengthSquared();
  MaxV(b);
  MinV(b);
  MulM(A);
  MulTM(A);
  Multiply(a);
  NegativeSelf();
  Normalize();
  Set(x_, y_);
  SetV(v);
  SetZero();
  Subtract(v);
  b2Vec2(x_, y_);

  static Make(x_, y_) => _b2Vec2.Make(x_, y_);
  static b2Vec2() => _b2Vec2.b2Vec2();
}

abstract class b2Contact {
  factory b2Contact() = _b2Contact;

  ComputeTOI(sweepA, sweepB);
  Evaluate();
  FlagForFiltering();
  GetFixtureA();
  GetFixtureB();
  GetManifold();
  GetNext();
  GetWorldManifold(worldManifold);
  IsContinuous();
  IsEnabled();
  IsSensor();
  IsTouching();
  Reset(fixtureA, fixtureB);
  SetEnabled(flag);
  SetSensor(sensor);
  Update(listener);
  b2Contact();

  static b2Contact() => _b2Contact.b2Contact();
}

abstract class b2WorldManifold {
  factory b2WorldManifold() = _b2WorldManifold;

  Initialize(manifold, xfA, radiusA, xfB, radiusB);
  b2WorldManifold();

  static b2WorldManifold() => _b2WorldManifold.b2WorldManifold();
}

abstract class b2PositionSolverManifold {
  factory b2PositionSolverManifold() = _b2PositionSolverManifold;

  Initialize(cc);
  b2PositionSolverManifold();

  static b2PositionSolverManifold() => _b2PositionSolverManifold.b2PositionSolverManifold();
}

abstract class b2TOIInput {
  factory b2TOIInput() = _b2TOIInput;

  static b2TOIInput() => _b2TOIInput.b2TOIInput();
}

abstract class b2DistanceProxy {
  factory b2DistanceProxy() = _b2DistanceProxy;

  GetSupport(d);
  GetSupportVertex(d);
  GetVertex(index);
  GetVertexCount();
  Set(shape);

  static b2DistanceProxy() => _b2DistanceProxy.b2DistanceProxy();
}

abstract class b2Shape {
  factory b2Shape() = _b2Shape;

  ComputeAABB(aabb, xf);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  Copy();
  GetType();
  RayCast(output, input, transform);
  Set(other);
  TestPoint(xf, p);
  b2Shape();

  static TestOverlap(shape1, transform1, shape2, transform2) => _b2Shape.TestOverlap(shape1, transform1, shape2, transform2);
  static b2Shape() => _b2Shape.b2Shape();
}

abstract class b2Mat22 {
  factory b2Mat22() = _b2Mat22;

  Abs();
  AddM(m);
  Copy();
  GetAngle();
  GetInverse(out);
  Set(angle);
  SetIdentity();
  SetM(m);
  SetVV(c1, c2);
  SetZero();
  Solve(out, bX, bY);
  b2Mat22();

  static FromAngle(angle) => _b2Mat22.FromAngle(angle);
  static FromVV(c1, c2) => _b2Mat22.FromVV(c1, c2);
  static b2Mat22() => _b2Mat22.b2Mat22();
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

  Evaluate(transformA, transformB);
  Initialize(cache, proxyA, transformA, proxyB, transformB);

  static b2SeparationFunction() => _b2SeparationFunction.b2SeparationFunction();
}

abstract class b2DistanceOutput {
  factory b2DistanceOutput() = _b2DistanceOutput;

  static b2DistanceOutput() => _b2DistanceOutput.b2DistanceOutput();
}

abstract class b2Simplex {
  factory b2Simplex() = _b2Simplex;

  GetClosestPoint();
  GetMetric();
  GetSearchDirection();
  GetWitnessPoints(pA, pB);
  ReadCache(cache, proxyA, transformA, proxyB, transformB);
  Solve2();
  Solve3();
  WriteCache(cache);
  b2Simplex();

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

  Copy();
  Set(id);
  b2ContactID();

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
  PostSolve(contact, impulse);
  PreSolve(contact, oldManifold);

  static b2ContactListener() => _b2ContactListener.b2ContactListener();
}

abstract class b2ContactFilter {
  factory b2ContactFilter() = _b2ContactFilter;

  RayCollide(userData, fixture);
  ShouldCollide(fixtureA, fixtureB);

  static b2ContactFilter() => _b2ContactFilter.b2ContactFilter();
}

abstract class b2JointDef {
  factory b2JointDef() = _b2JointDef;

  b2JointDef();

  static b2JointDef() => _b2JointDef.b2JointDef();
}

abstract class b2Joint {
  factory b2Joint(def) = _b2Joint;

  FinalizeVelocityConstraints();
  GetAnchorA();
  GetAnchorB();
  GetBodyA();
  GetBodyB();
  GetNext();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetType();
  GetUserData();
  InitVelocityConstraints(step);
  IsActive();
  SetUserData(data);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2Joint(def);

  static Create(def, allocator) => _b2Joint.Create(def, allocator);
  static Destroy(joint, allocator) => _b2Joint.Destroy(joint, allocator);
  static b2Joint() => _b2Joint.b2Joint();
}

abstract class b2Mat33 {
  factory b2Mat33(c1, c2, c3) = _b2Mat33;

  AddM(m);
  Copy();
  SetIdentity();
  SetM(m);
  SetVVV(c1, c2, c3);
  SetZero();
  Solve22(out, bX, bY);
  Solve33(out, bX, bY, bZ);
  b2Mat33(c1, c2, c3);

  static b2Mat33() => _b2Mat33.b2Mat33();
}

abstract class b2Vec3 {
  factory b2Vec3(x, y, z) = _b2Vec3;

  Add(v);
  Copy();
  GetNegative();
  Multiply(a);
  NegativeSelf();
  Set(x, y, z);
  SetV(v);
  SetZero();
  Subtract(v);
  b2Vec3(x, y, z);

  static b2Vec3() => _b2Vec3.b2Vec3();
}

abstract class b2DistanceJoint extends b2Joint {
  factory b2DistanceJoint(def) = _b2DistanceJoint;

  GetAnchorA();
  GetAnchorB();
  GetDampingRatio();
  GetFrequency();
  GetLength();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  InitVelocityConstraints(step);
  SetDampingRatio(ratio);
  SetFrequency(hz);
  SetLength(length);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2DistanceJoint(def);

  static b2DistanceJoint() => _b2DistanceJoint.b2DistanceJoint();
}

abstract class b2DistanceJointDef extends b2JointDef {
  factory b2DistanceJointDef() = _b2DistanceJointDef;

  Initialize(bA, bB, anchorA, anchorB);
  b2DistanceJointDef();

  static b2DistanceJointDef() => _b2DistanceJointDef.b2DistanceJointDef();
}

abstract class b2FrictionJoint extends b2Joint {
  factory b2FrictionJoint(def) = _b2FrictionJoint;

  GetAnchorA();
  GetAnchorB();
  GetMaxForce();
  GetMaxTorque();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  InitVelocityConstraints(step);
  SetMaxForce(force);
  SetMaxTorque(torque);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2FrictionJoint(def);

  static b2FrictionJoint() => _b2FrictionJoint.b2FrictionJoint();
}

abstract class b2FrictionJointDef extends b2JointDef {
  factory b2FrictionJointDef() = _b2FrictionJointDef;

  Initialize(bA, bB, anchor);
  b2FrictionJointDef();

  static b2FrictionJointDef() => _b2FrictionJointDef.b2FrictionJointDef();
}

abstract class b2GearJoint extends b2Joint {
  factory b2GearJoint(def) = _b2GearJoint;

  GetAnchorA();
  GetAnchorB();
  GetRatio();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  InitVelocityConstraints(step);
  SetRatio(ratio);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2GearJoint(def);

  static b2GearJoint() => _b2GearJoint.b2GearJoint();
}

abstract class b2GearJointDef extends b2JointDef {
  factory b2GearJointDef() = _b2GearJointDef;

  b2GearJointDef();

  static b2GearJointDef() => _b2GearJointDef.b2GearJointDef();
}

abstract class b2Jacobian {
  factory b2Jacobian() = _b2Jacobian;

  Compute(x1, a1, x2, a2);
  Set(x1, a1, x2, a2);
  SetZero();

  static b2Jacobian() => _b2Jacobian.b2Jacobian();
}

abstract class b2LineJoint extends b2Joint {
  factory b2LineJoint(def) = _b2LineJoint;

  EnableLimit(flag);
  EnableMotor(flag);
  GetAnchorA();
  GetAnchorB();
  GetJointSpeed();
  GetJointTranslation();
  GetLowerLimit();
  GetMaxMotorForce();
  GetMotorForce();
  GetMotorSpeed();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetUpperLimit();
  InitVelocityConstraints(step);
  IsLimitEnabled();
  IsMotorEnabled();
  SetLimits(lower, upper);
  SetMaxMotorForce(force);
  SetMotorSpeed(speed);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2LineJoint(def);

  static b2LineJoint() => _b2LineJoint.b2LineJoint();
}

abstract class b2LineJointDef extends b2JointDef {
  factory b2LineJointDef() = _b2LineJointDef;

  Initialize(bA, bB, anchor, axis);
  b2LineJointDef();

  static b2LineJointDef() => _b2LineJointDef.b2LineJointDef();
}

abstract class b2MouseJoint extends b2Joint {
  factory b2MouseJoint(def) = _b2MouseJoint;

  GetAnchorA();
  GetAnchorB();
  GetDampingRatio();
  GetFrequency();
  GetMaxForce();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetTarget();
  InitVelocityConstraints(step);
  SetDampingRatio(ratio);
  SetFrequency(hz);
  SetMaxForce(maxForce);
  SetTarget(target);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2MouseJoint(def);

  static b2MouseJoint() => _b2MouseJoint.b2MouseJoint();
}

abstract class b2MouseJointDef extends b2JointDef {
  factory b2MouseJointDef() = _b2MouseJointDef;

  b2MouseJointDef();

  static b2MouseJointDef() => _b2MouseJointDef.b2MouseJointDef();
}

abstract class b2PrismaticJoint extends b2Joint {
  factory b2PrismaticJoint(def) = _b2PrismaticJoint;

  EnableLimit(flag);
  EnableMotor(flag);
  GetAnchorA();
  GetAnchorB();
  GetJointSpeed();
  GetJointTranslation();
  GetLowerLimit();
  GetMotorForce();
  GetMotorSpeed();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetUpperLimit();
  InitVelocityConstraints(step);
  IsLimitEnabled();
  IsMotorEnabled();
  SetLimits(lower, upper);
  SetMaxMotorForce(force);
  SetMotorSpeed(speed);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2PrismaticJoint(def);

  static b2PrismaticJoint() => _b2PrismaticJoint.b2PrismaticJoint();
}

abstract class b2PrismaticJointDef extends b2JointDef {
  factory b2PrismaticJointDef() = _b2PrismaticJointDef;

  Initialize(bA, bB, anchor, axis);
  b2PrismaticJointDef();

  static b2PrismaticJointDef() => _b2PrismaticJointDef.b2PrismaticJointDef();
}

abstract class b2PulleyJoint extends b2Joint {
  factory b2PulleyJoint(def) = _b2PulleyJoint;

  GetAnchorA();
  GetAnchorB();
  GetGroundAnchorA();
  GetGroundAnchorB();
  GetLength1();
  GetLength2();
  GetRatio();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  InitVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2PulleyJoint(def);

  static b2PulleyJoint() => _b2PulleyJoint.b2PulleyJoint();
}

abstract class b2PulleyJointDef extends b2JointDef {
  factory b2PulleyJointDef() = _b2PulleyJointDef;

  Initialize(bA, bB, gaA, gaB, anchorA, anchorB, r);
  b2PulleyJointDef();

  static b2PulleyJointDef() => _b2PulleyJointDef.b2PulleyJointDef();
}

abstract class b2RevoluteJoint extends b2Joint {
  factory b2RevoluteJoint(def) = _b2RevoluteJoint;

  EnableLimit(flag);
  EnableMotor(flag);
  GetAnchorA();
  GetAnchorB();
  GetJointAngle();
  GetJointSpeed();
  GetLowerLimit();
  GetMotorSpeed();
  GetMotorTorque();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  GetUpperLimit();
  InitVelocityConstraints(step);
  IsLimitEnabled();
  IsMotorEnabled();
  SetLimits(lower, upper);
  SetMaxMotorTorque(torque);
  SetMotorSpeed(speed);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2RevoluteJoint(def);

  static b2RevoluteJoint() => _b2RevoluteJoint.b2RevoluteJoint();
}

abstract class b2RevoluteJointDef extends b2JointDef {
  factory b2RevoluteJointDef() = _b2RevoluteJointDef;

  Initialize(bA, bB, anchor);
  b2RevoluteJointDef();

  static b2RevoluteJointDef() => _b2RevoluteJointDef.b2RevoluteJointDef();
}

abstract class b2WeldJoint extends b2Joint {
  factory b2WeldJoint(def) = _b2WeldJoint;

  GetAnchorA();
  GetAnchorB();
  GetReactionForce(inv_dt);
  GetReactionTorque(inv_dt);
  InitVelocityConstraints(step);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints(step);
  b2WeldJoint(def);

  static b2WeldJoint() => _b2WeldJoint.b2WeldJoint();
}

abstract class b2WeldJointDef extends b2JointDef {
  factory b2WeldJointDef() = _b2WeldJointDef;

  Initialize(bA, bB, anchor);
  b2WeldJointDef();

  static b2WeldJointDef() => _b2WeldJointDef.b2WeldJointDef();
}

abstract class b2Body {
  factory b2Body(bd, world) = _b2Body;

  Advance(t);
  ApplyForce(force, point);
  ApplyImpulse(impulse, point);
  ApplyTorque(torque);
  CreateFixture(def);
  CreateFixture2(shape, density);
  DestroyFixture(fixture);
  GetAngle();
  GetAngularDamping();
  GetAngularVelocity();
  GetContactList();
  GetControllerList();
  GetDefinition();
  GetFixtureList();
  GetInertia();
  GetJointList();
  GetLinearDamping();
  GetLinearVelocity();
  GetLinearVelocityFromLocalPoint(localPoint);
  GetLinearVelocityFromWorldPoint(worldPoint);
  GetLocalCenter();
  GetLocalPoint(worldPoint);
  GetLocalVector(worldVector);
  GetMass();
  GetMassData(data);
  GetNext();
  GetPosition();
  GetTransform();
  GetType();
  GetUserData();
  GetWorld();
  GetWorldCenter();
  GetWorldPoint(localPoint);
  GetWorldVector(localVector);
  IsActive();
  IsAwake();
  IsBullet();
  IsFixedRotation();
  IsSleepingAllowed();
  Merge(other);
  ResetMassData();
  SetActive(flag);
  SetAngle(angle);
  SetAngularDamping(angularDamping);
  SetAngularVelocity(omega);
  SetAwake(flag);
  SetBullet(flag);
  SetFixedRotation(fixed);
  SetLinearDamping(linearDamping);
  SetLinearVelocity(v);
  SetMassData(massData);
  SetPosition(position);
  SetPositionAndAngle(position, angle);
  SetSleepingAllowed(flag);
  SetTransform(xf);
  SetType(type);
  SetUserData(data);
  ShouldCollide(other);
  Split(callback);
  SynchronizeFixtures();
  SynchronizeTransform();
  b2Body(bd, world);
  connectEdges(s1, s2, angle1);

  static b2Body() => _b2Body.b2Body();
}

abstract class b2BodyDef {
  factory b2BodyDef() = _b2BodyDef;

  b2BodyDef();

  static b2BodyDef() => _b2BodyDef.b2BodyDef();
}

abstract class b2ContactManager {
  factory b2ContactManager() = _b2ContactManager;

  AddPair(proxyUserDataA, proxyUserDataB);
  Collide();
  Destroy(c);
  FindNewContacts();
  b2ContactManager();

  static b2ContactManager() => _b2ContactManager.b2ContactManager();
}

abstract class b2DebugDraw {
  factory b2DebugDraw() = _b2DebugDraw;

  AppendFlags(flags);
  ClearFlags(flags);
  DrawCircle(center, radius, color);
  DrawPolygon(vertices, vertexCount, color);
  DrawSegment(p1, p2, color);
  DrawSolidCircle(center, radius, axis, color);
  DrawSolidPolygon(vertices, vertexCount, color);
  DrawTransform(xf);
  GetAlpha();
  GetDrawScale();
  GetFillAlpha();
  GetFlags();
  GetLineThickness();
  GetSprite();
  GetXFormScale();
  SetAlpha(alpha);
  SetDrawScale(drawScale);
  SetFillAlpha(alpha);
  SetFlags(flags);
  SetLineThickness(lineThickness);
  SetSprite(sprite);
  SetXFormScale(xformScale);
  b2DebugDraw();

  static b2DebugDraw() => _b2DebugDraw.b2DebugDraw();
}

abstract class b2DestructionListener {
  factory b2DestructionListener() = _b2DestructionListener;

  SayGoodbyeFixture(fixture);
  SayGoodbyeJoint(joint);

  static b2DestructionListener() => _b2DestructionListener.b2DestructionListener();
}

abstract class b2FilterData {
  factory b2FilterData() = _b2FilterData;

  Copy();

  static b2FilterData() => _b2FilterData.b2FilterData();
}

abstract class b2Fixture {
  factory b2Fixture() = _b2Fixture;

  Create(body, xf, def);
  CreateProxy(broadPhase, xf);
  Destroy();
  DestroyProxy(broadPhase);
  GetAABB();
  GetBody();
  GetDensity();
  GetFilterData();
  GetFriction();
  GetMassData(massData);
  GetNext();
  GetRestitution();
  GetShape();
  GetType();
  GetUserData();
  IsSensor();
  RayCast(output, input);
  SetDensity(density);
  SetFilterData(filter);
  SetFriction(friction);
  SetRestitution(restitution);
  SetSensor(sensor);
  SetUserData(data);
  Synchronize(broadPhase, transform1, transform2);
  TestPoint(p);
  b2Fixture();

  static b2Fixture() => _b2Fixture.b2Fixture();
}

abstract class b2FixtureDef {
  factory b2FixtureDef() = _b2FixtureDef;

  b2FixtureDef();

  static b2FixtureDef() => _b2FixtureDef.b2FixtureDef();
}

abstract class b2Island {
  factory b2Island() = _b2Island;

  AddBody(body);
  AddContact(contact);
  AddJoint(joint);
  Clear();
  Initialize(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver);
  Report(constraints);
  Solve(step, gravity, allowSleep);
  SolveTOI(subStep);
  b2Island();

  static b2Island() => _b2Island.b2Island();
}

abstract class b2World {
  factory b2World(gravity, doSleep) = _b2World;

  AddController(c);
  ClearForces();
  CreateBody(def);
  CreateController(controller);
  CreateJoint(def);
  DestroyBody(b);
  DestroyController(controller);
  DestroyJoint(j);
  DrawDebugData();
  DrawJoint(joint);
  DrawShape(shape, xf, color);
  GetBodyCount();
  GetBodyList();
  GetContactCount();
  GetContactList();
  GetGravity();
  GetGroundBody();
  GetJointCount();
  GetJointList();
  GetProxyCount();
  IsLocked();
  QueryAABB(callback, aabb);
  QueryPoint(callback, p);
  QueryShape(callback, shape, transform);
  RayCast(callback, point1, point2);
  RayCastAll(point1, point2);
  RayCastOne(point1, point2);
  RemoveController(c);
  SetBroadPhase(broadPhase);
  SetContactFilter(filter);
  SetContactListener(listener);
  SetContinuousPhysics(flag);
  SetDebugDraw(debugDraw);
  SetDestructionListener(listener);
  SetGravity(gravity);
  SetWarmStarting(flag);
  Solve(step);
  SolveTOI(step);
  Step(dt, velocityIterations, positionIterations);
  Validate();
  b2World(gravity, doSleep);

  static b2World() => _b2World.b2World();
}

abstract class b2AABB {
  factory b2AABB() = _b2AABB;

  Combine(aabb1, aabb2);
  Contains(aabb);
  GetCenter();
  GetExtents();
  IsValid();
  RayCast(output, input);
  TestOverlap(other);

  static Combine(aabb1, aabb2) => _b2AABB.Combine(aabb1, aabb2);
  static b2AABB() => _b2AABB.b2AABB();
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

  AllocateNode();
  CreateProxy(aabb, userData);
  DestroyProxy(proxy);
  FreeNode(node);
  GetFatAABB(proxy);
  GetUserData(proxy);
  InsertLeaf(leaf);
  MoveProxy(proxy, aabb, displacement);
  Query(callback, aabb);
  RayCast(callback, input);
  Rebalance(iterations);
  RemoveLeaf(leaf);
  b2DynamicTree();

  static b2DynamicTree() => _b2DynamicTree.b2DynamicTree();
}

abstract class b2DynamicTreeBroadPhase {
  factory b2DynamicTreeBroadPhase() = _b2DynamicTreeBroadPhase;

  BufferMove(proxy);
  ComparePairs(pair1, pair2);
  CreateProxy(aabb, userData);
  DestroyProxy(proxy);
  GetFatAABB(proxy);
  GetProxyCount();
  GetUserData(proxy);
  MoveProxy(proxy, aabb, displacement);
  Query(callback, aabb);
  RayCast(callback, input);
  Rebalance(iterations);
  TestOverlap(proxyA, proxyB);
  UnBufferMove(proxy);
  UpdatePairs(callback);
  Validate();

  static b2DynamicTreeBroadPhase() => _b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase();
}

abstract class b2DynamicTreeNode {
  factory b2DynamicTreeNode() = _b2DynamicTreeNode;

  IsLeaf();

  static b2DynamicTreeNode() => _b2DynamicTreeNode.b2DynamicTreeNode();
}

abstract class b2Manifold {
  factory b2Manifold() = _b2Manifold;

  Copy();
  Reset();
  Set(m);
  b2Manifold();

  static b2Manifold() => _b2Manifold.b2Manifold();
}

abstract class b2ManifoldPoint {
  factory b2ManifoldPoint() = _b2ManifoldPoint;

  Reset();
  Set(m);
  b2ManifoldPoint();

  static b2ManifoldPoint() => _b2ManifoldPoint.b2ManifoldPoint();
}

abstract class b2Point {
  factory b2Point() = _b2Point;

  GetFirstVertex(xf);
  Support(xf, vX, vY);

  static b2Point() => _b2Point.b2Point();
}

abstract class b2RayCastInput {
  factory b2RayCastInput(p1, p2, maxFraction) = _b2RayCastInput;

  b2RayCastInput(p1, p2, maxFraction);

  static b2RayCastInput() => _b2RayCastInput.b2RayCastInput();
}

abstract class b2Segment {
  factory b2Segment() = _b2Segment;

  Extend(aabb);
  ExtendBackward(aabb);
  ExtendForward(aabb);
  TestSegment(lambda, normal, segment, maxLambda);

  static b2Segment() => _b2Segment.b2Segment();
}

abstract class b2CircleShape extends b2Shape {
  factory b2CircleShape(radius) = _b2CircleShape;

  ComputeAABB(aabb, transform);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  Copy();
  GetLocalPosition();
  GetRadius();
  RayCast(output, input, transform);
  Set(other);
  SetLocalPosition(position);
  SetRadius(radius);
  TestPoint(transform, p);
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

  ComputeAABB(aabb, transform);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  Corner1IsConvex();
  Corner2IsConvex();
  GetCoreVertex1();
  GetCoreVertex2();
  GetCorner1Vector();
  GetCorner2Vector();
  GetDirectionVector();
  GetFirstVertex(xf);
  GetLength();
  GetNextEdge();
  GetNormalVector();
  GetPrevEdge();
  GetVertex1();
  GetVertex2();
  RayCast(output, input, transform);
  SetNextEdge(edge, core, cornerDir, convex);
  SetPrevEdge(edge, core, cornerDir, convex);
  Support(xf, dX, dY);
  TestPoint(transform, p);
  b2EdgeShape(v1, v2);

  static b2EdgeShape() => _b2EdgeShape.b2EdgeShape();
}

abstract class b2PolygonShape extends b2Shape {
  factory b2PolygonShape() = _b2PolygonShape;

  ComputeAABB(aabb, xf);
  ComputeMass(massData, density);
  ComputeSubmergedArea(normal, offset, xf, c);
  Copy();
  GetNormals();
  GetSupport(d);
  GetSupportVertex(d);
  GetVertexCount();
  GetVertices();
  RayCast(output, input, transform);
  Reserve(count);
  Set(other);
  SetAsArray(vertices, vertexCount);
  SetAsBox(hx, hy);
  SetAsEdge(v1, v2);
  SetAsOrientedBox(hx, hy, center, angle);
  SetAsVector(vertices, vertexCount);
  TestPoint(xf, p);
  Validate();
  b2PolygonShape();

  static AsArray(vertices, vertexCount) => _b2PolygonShape.AsArray(vertices, vertexCount);
  static AsBox(hx, hy) => _b2PolygonShape.AsBox(hx, hy);
  static AsEdge(v1, v2) => _b2PolygonShape.AsEdge(v1, v2);
  static AsOrientedBox(hx, hy, center, angle) => _b2PolygonShape.AsOrientedBox(hx, hy, center, angle);
  static AsVector(vertices, vertexCount) => _b2PolygonShape.AsVector(vertices, vertexCount);
  static ComputeCentroid(vs, count) => _b2PolygonShape.ComputeCentroid(vs, count);
  static ComputeOBB(obb, vs, count) => _b2PolygonShape.ComputeOBB(obb, vs, count);
  static b2PolygonShape() => _b2PolygonShape.b2PolygonShape();
}

abstract class b2CircleContact extends b2Contact {
  factory b2CircleContact() = _b2CircleContact;

  Evaluate();
  Reset(fixtureA, fixtureB);

  static Create(allocator) => _b2CircleContact.Create(allocator);
  static Destroy(contact, allocator) => _b2CircleContact.Destroy(contact, allocator);
  static b2CircleContact() => _b2CircleContact.b2CircleContact();
}

abstract class b2ContactConstraint {
  factory b2ContactConstraint() = _b2ContactConstraint;

  b2ContactConstraint();

  static b2ContactConstraint() => _b2ContactConstraint.b2ContactConstraint();
}

abstract class b2ContactFactory {
  factory b2ContactFactory(allocator) = _b2ContactFactory;

  AddType(createFcn, destroyFcn, type1, type2);
  Create(fixtureA, fixtureB);
  Destroy(contact);
  InitializeRegisters();
  b2ContactFactory(allocator);

  static b2ContactFactory() => _b2ContactFactory.b2ContactFactory();
}

abstract class b2ContactSolver {
  factory b2ContactSolver() = _b2ContactSolver;

  FinalizeVelocityConstraints();
  InitVelocityConstraints(step);
  Initialize(step, contacts, contactCount, allocator);
  SolvePositionConstraints(baumgarte);
  SolveVelocityConstraints();
  b2ContactSolver();

  static b2ContactSolver() => _b2ContactSolver.b2ContactSolver();
}

abstract class b2EdgeAndCircleContact extends b2Contact {
  factory b2EdgeAndCircleContact() = _b2EdgeAndCircleContact;

  Evaluate();
  Reset(fixtureA, fixtureB);
  b2CollideEdgeAndCircle(manifold, edge, xf1, circle, xf2);

  static Create(allocator) => _b2EdgeAndCircleContact.Create(allocator);
  static Destroy(contact, allocator) => _b2EdgeAndCircleContact.Destroy(contact, allocator);
  static b2EdgeAndCircleContact() => _b2EdgeAndCircleContact.b2EdgeAndCircleContact();
}

abstract class b2NullContact extends b2Contact {
  factory b2NullContact() = _b2NullContact;

  Evaluate();
  b2NullContact();

  static b2NullContact() => _b2NullContact.b2NullContact();
}

abstract class b2PolyAndCircleContact extends b2Contact {
  factory b2PolyAndCircleContact() = _b2PolyAndCircleContact;

  Evaluate();
  Reset(fixtureA, fixtureB);

  static Create(allocator) => _b2PolyAndCircleContact.Create(allocator);
  static Destroy(contact, allocator) => _b2PolyAndCircleContact.Destroy(contact, allocator);
  static b2PolyAndCircleContact() => _b2PolyAndCircleContact.b2PolyAndCircleContact();
}

abstract class b2PolyAndEdgeContact extends b2Contact {
  factory b2PolyAndEdgeContact() = _b2PolyAndEdgeContact;

  Evaluate();
  Reset(fixtureA, fixtureB);
  b2CollidePolyAndEdge(manifold, polygon, xf1, edge, xf2);

  static Create(allocator) => _b2PolyAndEdgeContact.Create(allocator);
  static Destroy(contact, allocator) => _b2PolyAndEdgeContact.Destroy(contact, allocator);
  static b2PolyAndEdgeContact() => _b2PolyAndEdgeContact.b2PolyAndEdgeContact();
}

abstract class b2PolygonContact extends b2Contact {
  factory b2PolygonContact() = _b2PolygonContact;

  Evaluate();
  Reset(fixtureA, fixtureB);

  static Create(allocator) => _b2PolygonContact.Create(allocator);
  static Destroy(contact, allocator) => _b2PolygonContact.Destroy(contact, allocator);
  static b2PolygonContact() => _b2PolygonContact.b2PolygonContact();
}

abstract class b2BuoyancyController extends b2Controller {
  factory b2BuoyancyController() = _b2BuoyancyController;

  Draw(debugDraw);
  Step(step);

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

  abstract class _Proxy {
    get jsvalue;
  }

  _(x) {
    if (x is _Proxy) return x.jsvalue;
    if (x is Map || x is Iterable) return new JsObject.jsify(x);
    return x;
  }

class _b2TimeStep extends _Proxy implements b2TimeStep {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2TimeStep"];
  _b2TimeStep() : jsvalue = new JsObject(_constructor, []);

  Set(step) => jsvalue.callMethod(r"Set", [_(step)]);
  static b2TimeStep() => _constructor.callMethod(r"b2TimeStep", []);
}

class _b2Transform extends _Proxy implements b2Transform {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Transform"];
  _b2Transform(pos, r) : jsvalue = new JsObject(_constructor, [_(pos), _(r)]);

  GetAngle() => jsvalue.callMethod(r"GetAngle", []);
  Initialize(pos, r) => jsvalue.callMethod(r"Initialize", [_(pos), _(r)]);
  Set(x) => jsvalue.callMethod(r"Set", [_(x)]);
  SetIdentity() => jsvalue.callMethod(r"SetIdentity", []);
  b2Transform(pos, r) => jsvalue.callMethod(r"b2Transform", [_(pos), _(r)]);
  static b2Transform() => _constructor.callMethod(r"b2Transform", []);
}

class _b2Sweep extends _Proxy implements b2Sweep {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Sweep"];
  _b2Sweep() : jsvalue = new JsObject(_constructor, []);

  Advance(t) => jsvalue.callMethod(r"Advance", [_(t)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  GetTransform(xf, alpha) => jsvalue.callMethod(r"GetTransform", [_(xf), _(alpha)]);
  Set(other) => jsvalue.callMethod(r"Set", [_(other)]);
  static b2Sweep() => _constructor.callMethod(r"b2Sweep", []);
}

class _b2Color extends _Proxy implements b2Color {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["b2Color"];
  _b2Color(rr, gg, bb) : jsvalue = new JsObject(_constructor, [_(rr), _(gg), _(bb)]);

  Set(rr, gg, bb) => jsvalue.callMethod(r"Set", [_(rr), _(gg), _(bb)]);
  b2Color(rr, gg, bb) => jsvalue.callMethod(r"b2Color", [_(rr), _(gg), _(bb)]);
  static b2Color() => _constructor.callMethod(r"b2Color", []);
}

class _b2Controller extends _Proxy implements b2Controller {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2Controller"];
  _b2Controller() : jsvalue = new JsObject(_constructor, []);
  _b2Controller.withValue(value) : jsvalue = value;

  AddBody(body) => jsvalue.callMethod(r"AddBody", [_(body)]);
  Clear() => jsvalue.callMethod(r"Clear", []);
  Draw(debugDraw) => jsvalue.callMethod(r"Draw", [_(debugDraw)]);
  GetBodyList() => jsvalue.callMethod(r"GetBodyList", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetWorld() => jsvalue.callMethod(r"GetWorld", []);
  RemoveBody(body) => jsvalue.callMethod(r"RemoveBody", [_(body)]);
  Step(step) => jsvalue.callMethod(r"Step", [_(step)]);
  static b2Controller() => _constructor.callMethod(r"b2Controller", []);
}

class _b2Vec2 extends _Proxy implements b2Vec2 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Vec2"];
  _b2Vec2(x_, y_) : jsvalue = new JsObject(_constructor, [_(x_), _(y_)]);

  Abs() => jsvalue.callMethod(r"Abs", []);
  Add(v) => jsvalue.callMethod(r"Add", [_(v)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  CrossFV(s) => jsvalue.callMethod(r"CrossFV", [_(s)]);
  CrossVF(s) => jsvalue.callMethod(r"CrossVF", [_(s)]);
  GetNegative() => jsvalue.callMethod(r"GetNegative", []);
  IsValid() => jsvalue.callMethod(r"IsValid", []);
  Length() => jsvalue.callMethod(r"Length", []);
  LengthSquared() => jsvalue.callMethod(r"LengthSquared", []);
  MaxV(b) => jsvalue.callMethod(r"MaxV", [_(b)]);
  MinV(b) => jsvalue.callMethod(r"MinV", [_(b)]);
  MulM(A) => jsvalue.callMethod(r"MulM", [_(A)]);
  MulTM(A) => jsvalue.callMethod(r"MulTM", [_(A)]);
  Multiply(a) => jsvalue.callMethod(r"Multiply", [_(a)]);
  NegativeSelf() => jsvalue.callMethod(r"NegativeSelf", []);
  Normalize() => jsvalue.callMethod(r"Normalize", []);
  Set(x_, y_) => jsvalue.callMethod(r"Set", [_(x_), _(y_)]);
  SetV(v) => jsvalue.callMethod(r"SetV", [_(v)]);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Subtract(v) => jsvalue.callMethod(r"Subtract", [_(v)]);
  b2Vec2(x_, y_) => jsvalue.callMethod(r"b2Vec2", [_(x_), _(y_)]);
  static Make(x_, y_) => _constructor.callMethod(r"Make", [_(x_), _(y_)]);
  static b2Vec2() => _constructor.callMethod(r"b2Vec2", []);
}

class _b2Contact extends _Proxy implements b2Contact {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2Contact"];
  _b2Contact() : jsvalue = new JsObject(_constructor, []);
  _b2Contact.withValue(value) : jsvalue = value;

  ComputeTOI(sweepA, sweepB) => jsvalue.callMethod(r"ComputeTOI", [_(sweepA), _(sweepB)]);
  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  FlagForFiltering() => jsvalue.callMethod(r"FlagForFiltering", []);
  GetFixtureA() => jsvalue.callMethod(r"GetFixtureA", []);
  GetFixtureB() => jsvalue.callMethod(r"GetFixtureB", []);
  GetManifold() => jsvalue.callMethod(r"GetManifold", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetWorldManifold(worldManifold) => jsvalue.callMethod(r"GetWorldManifold", [_(worldManifold)]);
  IsContinuous() => jsvalue.callMethod(r"IsContinuous", []);
  IsEnabled() => jsvalue.callMethod(r"IsEnabled", []);
  IsSensor() => jsvalue.callMethod(r"IsSensor", []);
  IsTouching() => jsvalue.callMethod(r"IsTouching", []);
  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [_(fixtureA), _(fixtureB)]);
  SetEnabled(flag) => jsvalue.callMethod(r"SetEnabled", [_(flag)]);
  SetSensor(sensor) => jsvalue.callMethod(r"SetSensor", [_(sensor)]);
  Update(listener) => jsvalue.callMethod(r"Update", [_(listener)]);
  b2Contact() => jsvalue.callMethod(r"b2Contact", []);
  static b2Contact() => _constructor.callMethod(r"b2Contact", []);
}

class _b2WorldManifold extends _Proxy implements b2WorldManifold {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2WorldManifold"];
  _b2WorldManifold() : jsvalue = new JsObject(_constructor, []);

  Initialize(manifold, xfA, radiusA, xfB, radiusB) => jsvalue.callMethod(r"Initialize", [_(manifold), _(xfA), _(radiusA), _(xfB), _(radiusB)]);
  b2WorldManifold() => jsvalue.callMethod(r"b2WorldManifold", []);
  static b2WorldManifold() => _constructor.callMethod(r"b2WorldManifold", []);
}

class _b2PositionSolverManifold extends _Proxy implements b2PositionSolverManifold {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PositionSolverManifold"];
  _b2PositionSolverManifold() : jsvalue = new JsObject(_constructor, []);

  Initialize(cc) => jsvalue.callMethod(r"Initialize", [_(cc)]);
  b2PositionSolverManifold() => jsvalue.callMethod(r"b2PositionSolverManifold", []);
  static b2PositionSolverManifold() => _constructor.callMethod(r"b2PositionSolverManifold", []);
}

class _b2TOIInput extends _Proxy implements b2TOIInput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2TOIInput"];
  _b2TOIInput() : jsvalue = new JsObject(_constructor, []);

  static b2TOIInput() => _constructor.callMethod(r"b2TOIInput", []);
}

class _b2DistanceProxy extends _Proxy implements b2DistanceProxy {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DistanceProxy"];
  _b2DistanceProxy() : jsvalue = new JsObject(_constructor, []);

  GetSupport(d) => jsvalue.callMethod(r"GetSupport", [_(d)]);
  GetSupportVertex(d) => jsvalue.callMethod(r"GetSupportVertex", [_(d)]);
  GetVertex(index) => jsvalue.callMethod(r"GetVertex", [_(index)]);
  GetVertexCount() => jsvalue.callMethod(r"GetVertexCount", []);
  Set(shape) => jsvalue.callMethod(r"Set", [_(shape)]);
  static b2DistanceProxy() => _constructor.callMethod(r"b2DistanceProxy", []);
}

class _b2Shape extends _Proxy implements b2Shape {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2Shape"];
  _b2Shape() : jsvalue = new JsObject(_constructor, []);
  _b2Shape.withValue(value) : jsvalue = value;

  ComputeAABB(aabb, xf) => jsvalue.callMethod(r"ComputeAABB", [_(aabb), _(xf)]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [_(massData), _(density)]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [_(normal), _(offset), _(xf), _(c)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  GetType() => jsvalue.callMethod(r"GetType", []);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [_(output), _(input), _(transform)]);
  Set(other) => jsvalue.callMethod(r"Set", [_(other)]);
  TestPoint(xf, p) => jsvalue.callMethod(r"TestPoint", [_(xf), _(p)]);
  b2Shape() => jsvalue.callMethod(r"b2Shape", []);
  static TestOverlap(shape1, transform1, shape2, transform2) => _constructor.callMethod(r"TestOverlap", [_(shape1), _(transform1), _(shape2), _(transform2)]);
  static b2Shape() => _constructor.callMethod(r"b2Shape", []);
}

class _b2Mat22 extends _Proxy implements b2Mat22 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Mat22"];
  _b2Mat22() : jsvalue = new JsObject(_constructor, []);

  Abs() => jsvalue.callMethod(r"Abs", []);
  AddM(m) => jsvalue.callMethod(r"AddM", [_(m)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  GetAngle() => jsvalue.callMethod(r"GetAngle", []);
  GetInverse(out) => jsvalue.callMethod(r"GetInverse", [_(out)]);
  Set(angle) => jsvalue.callMethod(r"Set", [_(angle)]);
  SetIdentity() => jsvalue.callMethod(r"SetIdentity", []);
  SetM(m) => jsvalue.callMethod(r"SetM", [_(m)]);
  SetVV(c1, c2) => jsvalue.callMethod(r"SetVV", [_(c1), _(c2)]);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Solve(out, bX, bY) => jsvalue.callMethod(r"Solve", [_(out), _(bX), _(bY)]);
  b2Mat22() => jsvalue.callMethod(r"b2Mat22", []);
  static FromAngle(angle) => _constructor.callMethod(r"FromAngle", [_(angle)]);
  static FromVV(c1, c2) => _constructor.callMethod(r"FromVV", [_(c1), _(c2)]);
  static b2Mat22() => _constructor.callMethod(r"b2Mat22", []);
}

class _b2SimplexCache extends _Proxy implements b2SimplexCache {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2SimplexCache"];
  _b2SimplexCache() : jsvalue = new JsObject(_constructor, []);

  static b2SimplexCache() => _constructor.callMethod(r"b2SimplexCache", []);
}

class _b2DistanceInput extends _Proxy implements b2DistanceInput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DistanceInput"];
  _b2DistanceInput() : jsvalue = new JsObject(_constructor, []);

  static b2DistanceInput() => _constructor.callMethod(r"b2DistanceInput", []);
}

class _b2SeparationFunction extends _Proxy implements b2SeparationFunction {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2SeparationFunction"];
  _b2SeparationFunction() : jsvalue = new JsObject(_constructor, []);

  Evaluate(transformA, transformB) => jsvalue.callMethod(r"Evaluate", [_(transformA), _(transformB)]);
  Initialize(cache, proxyA, transformA, proxyB, transformB) => jsvalue.callMethod(r"Initialize", [_(cache), _(proxyA), _(transformA), _(proxyB), _(transformB)]);
  static b2SeparationFunction() => _constructor.callMethod(r"b2SeparationFunction", []);
}

class _b2DistanceOutput extends _Proxy implements b2DistanceOutput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DistanceOutput"];
  _b2DistanceOutput() : jsvalue = new JsObject(_constructor, []);

  static b2DistanceOutput() => _constructor.callMethod(r"b2DistanceOutput", []);
}

class _b2Simplex extends _Proxy implements b2Simplex {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Simplex"];
  _b2Simplex() : jsvalue = new JsObject(_constructor, []);

  GetClosestPoint() => jsvalue.callMethod(r"GetClosestPoint", []);
  GetMetric() => jsvalue.callMethod(r"GetMetric", []);
  GetSearchDirection() => jsvalue.callMethod(r"GetSearchDirection", []);
  GetWitnessPoints(pA, pB) => jsvalue.callMethod(r"GetWitnessPoints", [_(pA), _(pB)]);
  ReadCache(cache, proxyA, transformA, proxyB, transformB) => jsvalue.callMethod(r"ReadCache", [_(cache), _(proxyA), _(transformA), _(proxyB), _(transformB)]);
  Solve2() => jsvalue.callMethod(r"Solve2", []);
  Solve3() => jsvalue.callMethod(r"Solve3", []);
  WriteCache(cache) => jsvalue.callMethod(r"WriteCache", [_(cache)]);
  b2Simplex() => jsvalue.callMethod(r"b2Simplex", []);
  static b2Simplex() => _constructor.callMethod(r"b2Simplex", []);
}

class _b2SimplexVertex extends _Proxy implements b2SimplexVertex {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2SimplexVertex"];
  _b2SimplexVertex() : jsvalue = new JsObject(_constructor, []);

  Set(other) => jsvalue.callMethod(r"Set", [_(other)]);
  static b2SimplexVertex() => _constructor.callMethod(r"b2SimplexVertex", []);
}

class _ClipVertex extends _Proxy implements ClipVertex {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["ClipVertex"];
  _ClipVertex() : jsvalue = new JsObject(_constructor, []);

  Set(other) => jsvalue.callMethod(r"Set", [_(other)]);
  static ClipVertex() => _constructor.callMethod(r"ClipVertex", []);
}

class _b2ContactID extends _Proxy implements b2ContactID {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2ContactID"];
  _b2ContactID() : jsvalue = new JsObject(_constructor, []);

  Copy() => jsvalue.callMethod(r"Copy", []);
  Set(id) => jsvalue.callMethod(r"Set", [_(id)]);
  b2ContactID() => jsvalue.callMethod(r"b2ContactID", []);
  static b2ContactID() => _constructor.callMethod(r"b2ContactID", []);
}

class _Features extends _Proxy implements Features {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Features"];
  _Features() : jsvalue = new JsObject(_constructor, []);

  static Features() => _constructor.callMethod(r"Features", []);
}

class _b2ContactImpulse extends _Proxy implements b2ContactImpulse {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactImpulse"];
  _b2ContactImpulse() : jsvalue = new JsObject(_constructor, []);

  static b2ContactImpulse() => _constructor.callMethod(r"b2ContactImpulse", []);
}

class _b2ContactPoint extends _Proxy implements b2ContactPoint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2ContactPoint"];
  _b2ContactPoint() : jsvalue = new JsObject(_constructor, []);

  static b2ContactPoint() => _constructor.callMethod(r"b2ContactPoint", []);
}

class _b2ContactListener extends _Proxy implements b2ContactListener {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactListener"];
  _b2ContactListener() : jsvalue = new JsObject(_constructor, []);

  BeginContact(contact) => jsvalue.callMethod(r"BeginContact", [_(contact)]);
  EndContact(contact) => jsvalue.callMethod(r"EndContact", [_(contact)]);
  PostSolve(contact, impulse) => jsvalue.callMethod(r"PostSolve", [_(contact), _(impulse)]);
  PreSolve(contact, oldManifold) => jsvalue.callMethod(r"PreSolve", [_(contact), _(oldManifold)]);
  static b2ContactListener() => _constructor.callMethod(r"b2ContactListener", []);
}

class _b2ContactFilter extends _Proxy implements b2ContactFilter {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactFilter"];
  _b2ContactFilter() : jsvalue = new JsObject(_constructor, []);

  RayCollide(userData, fixture) => jsvalue.callMethod(r"RayCollide", [_(userData), _(fixture)]);
  ShouldCollide(fixtureA, fixtureB) => jsvalue.callMethod(r"ShouldCollide", [_(fixtureA), _(fixtureB)]);
  static b2ContactFilter() => _constructor.callMethod(r"b2ContactFilter", []);
}

class _b2JointDef extends _Proxy implements b2JointDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2JointDef"];
  _b2JointDef() : jsvalue = new JsObject(_constructor, []);
  _b2JointDef.withValue(value) : jsvalue = value;

  b2JointDef() => jsvalue.callMethod(r"b2JointDef", []);
  static b2JointDef() => _constructor.callMethod(r"b2JointDef", []);
}

class _b2Joint extends _Proxy implements b2Joint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2Joint"];
  _b2Joint(def) : jsvalue = new JsObject(_constructor, [_(def)]);
  _b2Joint.withValue(value) : jsvalue = value;

  FinalizeVelocityConstraints() => jsvalue.callMethod(r"FinalizeVelocityConstraints", []);
  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetBodyA() => jsvalue.callMethod(r"GetBodyA", []);
  GetBodyB() => jsvalue.callMethod(r"GetBodyB", []);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  GetType() => jsvalue.callMethod(r"GetType", []);
  GetUserData() => jsvalue.callMethod(r"GetUserData", []);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  IsActive() => jsvalue.callMethod(r"IsActive", []);
  SetUserData(data) => jsvalue.callMethod(r"SetUserData", [_(data)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2Joint(def) => jsvalue.callMethod(r"b2Joint", [_(def)]);
  static Create(def, allocator) => _constructor.callMethod(r"Create", [_(def), _(allocator)]);
  static Destroy(joint, allocator) => _constructor.callMethod(r"Destroy", [_(joint), _(allocator)]);
  static b2Joint() => _constructor.callMethod(r"b2Joint", []);
}

class _b2Mat33 extends _Proxy implements b2Mat33 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Mat33"];
  _b2Mat33(c1, c2, c3) : jsvalue = new JsObject(_constructor, [_(c1), _(c2), _(c3)]);

  AddM(m) => jsvalue.callMethod(r"AddM", [_(m)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  SetIdentity() => jsvalue.callMethod(r"SetIdentity", []);
  SetM(m) => jsvalue.callMethod(r"SetM", [_(m)]);
  SetVVV(c1, c2, c3) => jsvalue.callMethod(r"SetVVV", [_(c1), _(c2), _(c3)]);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Solve22(out, bX, bY) => jsvalue.callMethod(r"Solve22", [_(out), _(bX), _(bY)]);
  Solve33(out, bX, bY, bZ) => jsvalue.callMethod(r"Solve33", [_(out), _(bX), _(bY), _(bZ)]);
  b2Mat33(c1, c2, c3) => jsvalue.callMethod(r"b2Mat33", [_(c1), _(c2), _(c3)]);
  static b2Mat33() => _constructor.callMethod(r"b2Mat33", []);
}

class _b2Vec3 extends _Proxy implements b2Vec3 {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Common"]["Math"]["b2Vec3"];
  _b2Vec3(x, y, z) : jsvalue = new JsObject(_constructor, [_(x), _(y), _(z)]);

  Add(v) => jsvalue.callMethod(r"Add", [_(v)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  GetNegative() => jsvalue.callMethod(r"GetNegative", []);
  Multiply(a) => jsvalue.callMethod(r"Multiply", [_(a)]);
  NegativeSelf() => jsvalue.callMethod(r"NegativeSelf", []);
  Set(x, y, z) => jsvalue.callMethod(r"Set", [_(x), _(y), _(z)]);
  SetV(v) => jsvalue.callMethod(r"SetV", [_(v)]);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  Subtract(v) => jsvalue.callMethod(r"Subtract", [_(v)]);
  b2Vec3(x, y, z) => jsvalue.callMethod(r"b2Vec3", [_(x), _(y), _(z)]);
  static b2Vec3() => _constructor.callMethod(r"b2Vec3", []);
}

class _b2DistanceJoint extends _b2Joint implements b2DistanceJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2DistanceJoint"];
  _b2DistanceJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetDampingRatio() => jsvalue.callMethod(r"GetDampingRatio", []);
  GetFrequency() => jsvalue.callMethod(r"GetFrequency", []);
  GetLength() => jsvalue.callMethod(r"GetLength", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  SetDampingRatio(ratio) => jsvalue.callMethod(r"SetDampingRatio", [_(ratio)]);
  SetFrequency(hz) => jsvalue.callMethod(r"SetFrequency", [_(hz)]);
  SetLength(length) => jsvalue.callMethod(r"SetLength", [_(length)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2DistanceJoint(def) => jsvalue.callMethod(r"b2DistanceJoint", [_(def)]);
  static b2DistanceJoint() => _constructor.callMethod(r"b2DistanceJoint", []);
}

class _b2DistanceJointDef extends _b2JointDef implements b2DistanceJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2DistanceJointDef"];
  _b2DistanceJointDef() : super.withValue(new JsObject(_constructor, []));

  Initialize(bA, bB, anchorA, anchorB) => jsvalue.callMethod(r"Initialize", [_(bA), _(bB), _(anchorA), _(anchorB)]);
  b2DistanceJointDef() => jsvalue.callMethod(r"b2DistanceJointDef", []);
  static b2DistanceJointDef() => _constructor.callMethod(r"b2DistanceJointDef", []);
}

class _b2FrictionJoint extends _b2Joint implements b2FrictionJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2FrictionJoint"];
  _b2FrictionJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetMaxForce() => jsvalue.callMethod(r"GetMaxForce", []);
  GetMaxTorque() => jsvalue.callMethod(r"GetMaxTorque", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  SetMaxForce(force) => jsvalue.callMethod(r"SetMaxForce", [_(force)]);
  SetMaxTorque(torque) => jsvalue.callMethod(r"SetMaxTorque", [_(torque)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2FrictionJoint(def) => jsvalue.callMethod(r"b2FrictionJoint", [_(def)]);
  static b2FrictionJoint() => _constructor.callMethod(r"b2FrictionJoint", []);
}

class _b2FrictionJointDef extends _b2JointDef implements b2FrictionJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2FrictionJointDef"];
  _b2FrictionJointDef() : super.withValue(new JsObject(_constructor, []));

  Initialize(bA, bB, anchor) => jsvalue.callMethod(r"Initialize", [_(bA), _(bB), _(anchor)]);
  b2FrictionJointDef() => jsvalue.callMethod(r"b2FrictionJointDef", []);
  static b2FrictionJointDef() => _constructor.callMethod(r"b2FrictionJointDef", []);
}

class _b2GearJoint extends _b2Joint implements b2GearJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2GearJoint"];
  _b2GearJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetRatio() => jsvalue.callMethod(r"GetRatio", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  SetRatio(ratio) => jsvalue.callMethod(r"SetRatio", [_(ratio)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2GearJoint(def) => jsvalue.callMethod(r"b2GearJoint", [_(def)]);
  static b2GearJoint() => _constructor.callMethod(r"b2GearJoint", []);
}

class _b2GearJointDef extends _b2JointDef implements b2GearJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2GearJointDef"];
  _b2GearJointDef() : super.withValue(new JsObject(_constructor, []));

  b2GearJointDef() => jsvalue.callMethod(r"b2GearJointDef", []);
  static b2GearJointDef() => _constructor.callMethod(r"b2GearJointDef", []);
}

class _b2Jacobian extends _Proxy implements b2Jacobian {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2Jacobian"];
  _b2Jacobian() : jsvalue = new JsObject(_constructor, []);

  Compute(x1, a1, x2, a2) => jsvalue.callMethod(r"Compute", [_(x1), _(a1), _(x2), _(a2)]);
  Set(x1, a1, x2, a2) => jsvalue.callMethod(r"Set", [_(x1), _(a1), _(x2), _(a2)]);
  SetZero() => jsvalue.callMethod(r"SetZero", []);
  static b2Jacobian() => _constructor.callMethod(r"b2Jacobian", []);
}

class _b2LineJoint extends _b2Joint implements b2LineJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2LineJoint"];
  _b2LineJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  EnableLimit(flag) => jsvalue.callMethod(r"EnableLimit", [_(flag)]);
  EnableMotor(flag) => jsvalue.callMethod(r"EnableMotor", [_(flag)]);
  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetJointSpeed() => jsvalue.callMethod(r"GetJointSpeed", []);
  GetJointTranslation() => jsvalue.callMethod(r"GetJointTranslation", []);
  GetLowerLimit() => jsvalue.callMethod(r"GetLowerLimit", []);
  GetMaxMotorForce() => jsvalue.callMethod(r"GetMaxMotorForce", []);
  GetMotorForce() => jsvalue.callMethod(r"GetMotorForce", []);
  GetMotorSpeed() => jsvalue.callMethod(r"GetMotorSpeed", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  GetUpperLimit() => jsvalue.callMethod(r"GetUpperLimit", []);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  IsLimitEnabled() => jsvalue.callMethod(r"IsLimitEnabled", []);
  IsMotorEnabled() => jsvalue.callMethod(r"IsMotorEnabled", []);
  SetLimits(lower, upper) => jsvalue.callMethod(r"SetLimits", [_(lower), _(upper)]);
  SetMaxMotorForce(force) => jsvalue.callMethod(r"SetMaxMotorForce", [_(force)]);
  SetMotorSpeed(speed) => jsvalue.callMethod(r"SetMotorSpeed", [_(speed)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2LineJoint(def) => jsvalue.callMethod(r"b2LineJoint", [_(def)]);
  static b2LineJoint() => _constructor.callMethod(r"b2LineJoint", []);
}

class _b2LineJointDef extends _b2JointDef implements b2LineJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2LineJointDef"];
  _b2LineJointDef() : super.withValue(new JsObject(_constructor, []));

  Initialize(bA, bB, anchor, axis) => jsvalue.callMethod(r"Initialize", [_(bA), _(bB), _(anchor), _(axis)]);
  b2LineJointDef() => jsvalue.callMethod(r"b2LineJointDef", []);
  static b2LineJointDef() => _constructor.callMethod(r"b2LineJointDef", []);
}

class _b2MouseJoint extends _b2Joint implements b2MouseJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2MouseJoint"];
  _b2MouseJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetDampingRatio() => jsvalue.callMethod(r"GetDampingRatio", []);
  GetFrequency() => jsvalue.callMethod(r"GetFrequency", []);
  GetMaxForce() => jsvalue.callMethod(r"GetMaxForce", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  GetTarget() => jsvalue.callMethod(r"GetTarget", []);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  SetDampingRatio(ratio) => jsvalue.callMethod(r"SetDampingRatio", [_(ratio)]);
  SetFrequency(hz) => jsvalue.callMethod(r"SetFrequency", [_(hz)]);
  SetMaxForce(maxForce) => jsvalue.callMethod(r"SetMaxForce", [_(maxForce)]);
  SetTarget(target) => jsvalue.callMethod(r"SetTarget", [_(target)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2MouseJoint(def) => jsvalue.callMethod(r"b2MouseJoint", [_(def)]);
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
  _b2PrismaticJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  EnableLimit(flag) => jsvalue.callMethod(r"EnableLimit", [_(flag)]);
  EnableMotor(flag) => jsvalue.callMethod(r"EnableMotor", [_(flag)]);
  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetJointSpeed() => jsvalue.callMethod(r"GetJointSpeed", []);
  GetJointTranslation() => jsvalue.callMethod(r"GetJointTranslation", []);
  GetLowerLimit() => jsvalue.callMethod(r"GetLowerLimit", []);
  GetMotorForce() => jsvalue.callMethod(r"GetMotorForce", []);
  GetMotorSpeed() => jsvalue.callMethod(r"GetMotorSpeed", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  GetUpperLimit() => jsvalue.callMethod(r"GetUpperLimit", []);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  IsLimitEnabled() => jsvalue.callMethod(r"IsLimitEnabled", []);
  IsMotorEnabled() => jsvalue.callMethod(r"IsMotorEnabled", []);
  SetLimits(lower, upper) => jsvalue.callMethod(r"SetLimits", [_(lower), _(upper)]);
  SetMaxMotorForce(force) => jsvalue.callMethod(r"SetMaxMotorForce", [_(force)]);
  SetMotorSpeed(speed) => jsvalue.callMethod(r"SetMotorSpeed", [_(speed)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2PrismaticJoint(def) => jsvalue.callMethod(r"b2PrismaticJoint", [_(def)]);
  static b2PrismaticJoint() => _constructor.callMethod(r"b2PrismaticJoint", []);
}

class _b2PrismaticJointDef extends _b2JointDef implements b2PrismaticJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2PrismaticJointDef"];
  _b2PrismaticJointDef() : super.withValue(new JsObject(_constructor, []));

  Initialize(bA, bB, anchor, axis) => jsvalue.callMethod(r"Initialize", [_(bA), _(bB), _(anchor), _(axis)]);
  b2PrismaticJointDef() => jsvalue.callMethod(r"b2PrismaticJointDef", []);
  static b2PrismaticJointDef() => _constructor.callMethod(r"b2PrismaticJointDef", []);
}

class _b2PulleyJoint extends _b2Joint implements b2PulleyJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2PulleyJoint"];
  _b2PulleyJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetGroundAnchorA() => jsvalue.callMethod(r"GetGroundAnchorA", []);
  GetGroundAnchorB() => jsvalue.callMethod(r"GetGroundAnchorB", []);
  GetLength1() => jsvalue.callMethod(r"GetLength1", []);
  GetLength2() => jsvalue.callMethod(r"GetLength2", []);
  GetRatio() => jsvalue.callMethod(r"GetRatio", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2PulleyJoint(def) => jsvalue.callMethod(r"b2PulleyJoint", [_(def)]);
  static b2PulleyJoint() => _constructor.callMethod(r"b2PulleyJoint", []);
}

class _b2PulleyJointDef extends _b2JointDef implements b2PulleyJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2PulleyJointDef"];
  _b2PulleyJointDef() : super.withValue(new JsObject(_constructor, []));

  Initialize(bA, bB, gaA, gaB, anchorA, anchorB, r) => jsvalue.callMethod(r"Initialize", [_(bA), _(bB), _(gaA), _(gaB), _(anchorA), _(anchorB), _(r)]);
  b2PulleyJointDef() => jsvalue.callMethod(r"b2PulleyJointDef", []);
  static b2PulleyJointDef() => _constructor.callMethod(r"b2PulleyJointDef", []);
}

class _b2RevoluteJoint extends _b2Joint implements b2RevoluteJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2RevoluteJoint"];
  _b2RevoluteJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  EnableLimit(flag) => jsvalue.callMethod(r"EnableLimit", [_(flag)]);
  EnableMotor(flag) => jsvalue.callMethod(r"EnableMotor", [_(flag)]);
  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetJointAngle() => jsvalue.callMethod(r"GetJointAngle", []);
  GetJointSpeed() => jsvalue.callMethod(r"GetJointSpeed", []);
  GetLowerLimit() => jsvalue.callMethod(r"GetLowerLimit", []);
  GetMotorSpeed() => jsvalue.callMethod(r"GetMotorSpeed", []);
  GetMotorTorque() => jsvalue.callMethod(r"GetMotorTorque", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  GetUpperLimit() => jsvalue.callMethod(r"GetUpperLimit", []);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  IsLimitEnabled() => jsvalue.callMethod(r"IsLimitEnabled", []);
  IsMotorEnabled() => jsvalue.callMethod(r"IsMotorEnabled", []);
  SetLimits(lower, upper) => jsvalue.callMethod(r"SetLimits", [_(lower), _(upper)]);
  SetMaxMotorTorque(torque) => jsvalue.callMethod(r"SetMaxMotorTorque", [_(torque)]);
  SetMotorSpeed(speed) => jsvalue.callMethod(r"SetMotorSpeed", [_(speed)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2RevoluteJoint(def) => jsvalue.callMethod(r"b2RevoluteJoint", [_(def)]);
  static b2RevoluteJoint() => _constructor.callMethod(r"b2RevoluteJoint", []);
}

class _b2RevoluteJointDef extends _b2JointDef implements b2RevoluteJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2RevoluteJointDef"];
  _b2RevoluteJointDef() : super.withValue(new JsObject(_constructor, []));

  Initialize(bA, bB, anchor) => jsvalue.callMethod(r"Initialize", [_(bA), _(bB), _(anchor)]);
  b2RevoluteJointDef() => jsvalue.callMethod(r"b2RevoluteJointDef", []);
  static b2RevoluteJointDef() => _constructor.callMethod(r"b2RevoluteJointDef", []);
}

class _b2WeldJoint extends _b2Joint implements b2WeldJoint {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2WeldJoint"];
  _b2WeldJoint(def) : super.withValue(new JsObject(_constructor, [_(def)]));

  GetAnchorA() => jsvalue.callMethod(r"GetAnchorA", []);
  GetAnchorB() => jsvalue.callMethod(r"GetAnchorB", []);
  GetReactionForce(inv_dt) => jsvalue.callMethod(r"GetReactionForce", [_(inv_dt)]);
  GetReactionTorque(inv_dt) => jsvalue.callMethod(r"GetReactionTorque", [_(inv_dt)]);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints(step) => jsvalue.callMethod(r"SolveVelocityConstraints", [_(step)]);
  b2WeldJoint(def) => jsvalue.callMethod(r"b2WeldJoint", [_(def)]);
  static b2WeldJoint() => _constructor.callMethod(r"b2WeldJoint", []);
}

class _b2WeldJointDef extends _b2JointDef implements b2WeldJointDef {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2WeldJointDef"];
  _b2WeldJointDef() : super.withValue(new JsObject(_constructor, []));

  Initialize(bA, bB, anchor) => jsvalue.callMethod(r"Initialize", [_(bA), _(bB), _(anchor)]);
  b2WeldJointDef() => jsvalue.callMethod(r"b2WeldJointDef", []);
  static b2WeldJointDef() => _constructor.callMethod(r"b2WeldJointDef", []);
}

class _b2Body extends _Proxy implements b2Body {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2Body"];
  _b2Body(bd, world) : jsvalue = new JsObject(_constructor, [_(bd), _(world)]);

  Advance(t) => jsvalue.callMethod(r"Advance", [_(t)]);
  ApplyForce(force, point) => jsvalue.callMethod(r"ApplyForce", [_(force), _(point)]);
  ApplyImpulse(impulse, point) => jsvalue.callMethod(r"ApplyImpulse", [_(impulse), _(point)]);
  ApplyTorque(torque) => jsvalue.callMethod(r"ApplyTorque", [_(torque)]);
  CreateFixture(def) => jsvalue.callMethod(r"CreateFixture", [_(def)]);
  CreateFixture2(shape, density) => jsvalue.callMethod(r"CreateFixture2", [_(shape), _(density)]);
  DestroyFixture(fixture) => jsvalue.callMethod(r"DestroyFixture", [_(fixture)]);
  GetAngle() => jsvalue.callMethod(r"GetAngle", []);
  GetAngularDamping() => jsvalue.callMethod(r"GetAngularDamping", []);
  GetAngularVelocity() => jsvalue.callMethod(r"GetAngularVelocity", []);
  GetContactList() => jsvalue.callMethod(r"GetContactList", []);
  GetControllerList() => jsvalue.callMethod(r"GetControllerList", []);
  GetDefinition() => jsvalue.callMethod(r"GetDefinition", []);
  GetFixtureList() => jsvalue.callMethod(r"GetFixtureList", []);
  GetInertia() => jsvalue.callMethod(r"GetInertia", []);
  GetJointList() => jsvalue.callMethod(r"GetJointList", []);
  GetLinearDamping() => jsvalue.callMethod(r"GetLinearDamping", []);
  GetLinearVelocity() => jsvalue.callMethod(r"GetLinearVelocity", []);
  GetLinearVelocityFromLocalPoint(localPoint) => jsvalue.callMethod(r"GetLinearVelocityFromLocalPoint", [_(localPoint)]);
  GetLinearVelocityFromWorldPoint(worldPoint) => jsvalue.callMethod(r"GetLinearVelocityFromWorldPoint", [_(worldPoint)]);
  GetLocalCenter() => jsvalue.callMethod(r"GetLocalCenter", []);
  GetLocalPoint(worldPoint) => jsvalue.callMethod(r"GetLocalPoint", [_(worldPoint)]);
  GetLocalVector(worldVector) => jsvalue.callMethod(r"GetLocalVector", [_(worldVector)]);
  GetMass() => jsvalue.callMethod(r"GetMass", []);
  GetMassData(data) => jsvalue.callMethod(r"GetMassData", [_(data)]);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetPosition() => jsvalue.callMethod(r"GetPosition", []);
  GetTransform() => jsvalue.callMethod(r"GetTransform", []);
  GetType() => jsvalue.callMethod(r"GetType", []);
  GetUserData() => jsvalue.callMethod(r"GetUserData", []);
  GetWorld() => jsvalue.callMethod(r"GetWorld", []);
  GetWorldCenter() => jsvalue.callMethod(r"GetWorldCenter", []);
  GetWorldPoint(localPoint) => jsvalue.callMethod(r"GetWorldPoint", [_(localPoint)]);
  GetWorldVector(localVector) => jsvalue.callMethod(r"GetWorldVector", [_(localVector)]);
  IsActive() => jsvalue.callMethod(r"IsActive", []);
  IsAwake() => jsvalue.callMethod(r"IsAwake", []);
  IsBullet() => jsvalue.callMethod(r"IsBullet", []);
  IsFixedRotation() => jsvalue.callMethod(r"IsFixedRotation", []);
  IsSleepingAllowed() => jsvalue.callMethod(r"IsSleepingAllowed", []);
  Merge(other) => jsvalue.callMethod(r"Merge", [_(other)]);
  ResetMassData() => jsvalue.callMethod(r"ResetMassData", []);
  SetActive(flag) => jsvalue.callMethod(r"SetActive", [_(flag)]);
  SetAngle(angle) => jsvalue.callMethod(r"SetAngle", [_(angle)]);
  SetAngularDamping(angularDamping) => jsvalue.callMethod(r"SetAngularDamping", [_(angularDamping)]);
  SetAngularVelocity(omega) => jsvalue.callMethod(r"SetAngularVelocity", [_(omega)]);
  SetAwake(flag) => jsvalue.callMethod(r"SetAwake", [_(flag)]);
  SetBullet(flag) => jsvalue.callMethod(r"SetBullet", [_(flag)]);
  SetFixedRotation(fixed) => jsvalue.callMethod(r"SetFixedRotation", [_(fixed)]);
  SetLinearDamping(linearDamping) => jsvalue.callMethod(r"SetLinearDamping", [_(linearDamping)]);
  SetLinearVelocity(v) => jsvalue.callMethod(r"SetLinearVelocity", [_(v)]);
  SetMassData(massData) => jsvalue.callMethod(r"SetMassData", [_(massData)]);
  SetPosition(position) => jsvalue.callMethod(r"SetPosition", [_(position)]);
  SetPositionAndAngle(position, angle) => jsvalue.callMethod(r"SetPositionAndAngle", [_(position), _(angle)]);
  SetSleepingAllowed(flag) => jsvalue.callMethod(r"SetSleepingAllowed", [_(flag)]);
  SetTransform(xf) => jsvalue.callMethod(r"SetTransform", [_(xf)]);
  SetType(type) => jsvalue.callMethod(r"SetType", [_(type)]);
  SetUserData(data) => jsvalue.callMethod(r"SetUserData", [_(data)]);
  ShouldCollide(other) => jsvalue.callMethod(r"ShouldCollide", [_(other)]);
  Split(callback) => jsvalue.callMethod(r"Split", [_(callback)]);
  SynchronizeFixtures() => jsvalue.callMethod(r"SynchronizeFixtures", []);
  SynchronizeTransform() => jsvalue.callMethod(r"SynchronizeTransform", []);
  b2Body(bd, world) => jsvalue.callMethod(r"b2Body", [_(bd), _(world)]);
  connectEdges(s1, s2, angle1) => jsvalue.callMethod(r"connectEdges", [_(s1), _(s2), _(angle1)]);
  static b2Body() => _constructor.callMethod(r"b2Body", []);
}

class _b2BodyDef extends _Proxy implements b2BodyDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2BodyDef"];
  _b2BodyDef() : jsvalue = new JsObject(_constructor, []);

  b2BodyDef() => jsvalue.callMethod(r"b2BodyDef", []);
  static b2BodyDef() => _constructor.callMethod(r"b2BodyDef", []);
}

class _b2ContactManager extends _Proxy implements b2ContactManager {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2ContactManager"];
  _b2ContactManager() : jsvalue = new JsObject(_constructor, []);

  AddPair(proxyUserDataA, proxyUserDataB) => jsvalue.callMethod(r"AddPair", [_(proxyUserDataA), _(proxyUserDataB)]);
  Collide() => jsvalue.callMethod(r"Collide", []);
  Destroy(c) => jsvalue.callMethod(r"Destroy", [_(c)]);
  FindNewContacts() => jsvalue.callMethod(r"FindNewContacts", []);
  b2ContactManager() => jsvalue.callMethod(r"b2ContactManager", []);
  static b2ContactManager() => _constructor.callMethod(r"b2ContactManager", []);
}

class _b2DebugDraw extends _Proxy implements b2DebugDraw {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2DebugDraw"];
  _b2DebugDraw() : jsvalue = new JsObject(_constructor, []);

  AppendFlags(flags) => jsvalue.callMethod(r"AppendFlags", [_(flags)]);
  ClearFlags(flags) => jsvalue.callMethod(r"ClearFlags", [_(flags)]);
  DrawCircle(center, radius, color) => jsvalue.callMethod(r"DrawCircle", [_(center), _(radius), _(color)]);
  DrawPolygon(vertices, vertexCount, color) => jsvalue.callMethod(r"DrawPolygon", [_(vertices), _(vertexCount), _(color)]);
  DrawSegment(p1, p2, color) => jsvalue.callMethod(r"DrawSegment", [_(p1), _(p2), _(color)]);
  DrawSolidCircle(center, radius, axis, color) => jsvalue.callMethod(r"DrawSolidCircle", [_(center), _(radius), _(axis), _(color)]);
  DrawSolidPolygon(vertices, vertexCount, color) => jsvalue.callMethod(r"DrawSolidPolygon", [_(vertices), _(vertexCount), _(color)]);
  DrawTransform(xf) => jsvalue.callMethod(r"DrawTransform", [_(xf)]);
  GetAlpha() => jsvalue.callMethod(r"GetAlpha", []);
  GetDrawScale() => jsvalue.callMethod(r"GetDrawScale", []);
  GetFillAlpha() => jsvalue.callMethod(r"GetFillAlpha", []);
  GetFlags() => jsvalue.callMethod(r"GetFlags", []);
  GetLineThickness() => jsvalue.callMethod(r"GetLineThickness", []);
  GetSprite() => jsvalue.callMethod(r"GetSprite", []);
  GetXFormScale() => jsvalue.callMethod(r"GetXFormScale", []);
  SetAlpha(alpha) => jsvalue.callMethod(r"SetAlpha", [_(alpha)]);
  SetDrawScale(drawScale) => jsvalue.callMethod(r"SetDrawScale", [_(drawScale)]);
  SetFillAlpha(alpha) => jsvalue.callMethod(r"SetFillAlpha", [_(alpha)]);
  SetFlags(flags) => jsvalue.callMethod(r"SetFlags", [_(flags)]);
  SetLineThickness(lineThickness) => jsvalue.callMethod(r"SetLineThickness", [_(lineThickness)]);
  SetSprite(sprite) => jsvalue.callMethod(r"SetSprite", [_(sprite)]);
  SetXFormScale(xformScale) => jsvalue.callMethod(r"SetXFormScale", [_(xformScale)]);
  b2DebugDraw() => jsvalue.callMethod(r"b2DebugDraw", []);
  static b2DebugDraw() => _constructor.callMethod(r"b2DebugDraw", []);
}

class _b2DestructionListener extends _Proxy implements b2DestructionListener {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2DestructionListener"];
  _b2DestructionListener() : jsvalue = new JsObject(_constructor, []);

  SayGoodbyeFixture(fixture) => jsvalue.callMethod(r"SayGoodbyeFixture", [_(fixture)]);
  SayGoodbyeJoint(joint) => jsvalue.callMethod(r"SayGoodbyeJoint", [_(joint)]);
  static b2DestructionListener() => _constructor.callMethod(r"b2DestructionListener", []);
}

class _b2FilterData extends _Proxy implements b2FilterData {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2FilterData"];
  _b2FilterData() : jsvalue = new JsObject(_constructor, []);

  Copy() => jsvalue.callMethod(r"Copy", []);
  static b2FilterData() => _constructor.callMethod(r"b2FilterData", []);
}

class _b2Fixture extends _Proxy implements b2Fixture {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2Fixture"];
  _b2Fixture() : jsvalue = new JsObject(_constructor, []);

  Create(body, xf, def) => jsvalue.callMethod(r"Create", [_(body), _(xf), _(def)]);
  CreateProxy(broadPhase, xf) => jsvalue.callMethod(r"CreateProxy", [_(broadPhase), _(xf)]);
  Destroy() => jsvalue.callMethod(r"Destroy", []);
  DestroyProxy(broadPhase) => jsvalue.callMethod(r"DestroyProxy", [_(broadPhase)]);
  GetAABB() => jsvalue.callMethod(r"GetAABB", []);
  GetBody() => jsvalue.callMethod(r"GetBody", []);
  GetDensity() => jsvalue.callMethod(r"GetDensity", []);
  GetFilterData() => jsvalue.callMethod(r"GetFilterData", []);
  GetFriction() => jsvalue.callMethod(r"GetFriction", []);
  GetMassData(massData) => jsvalue.callMethod(r"GetMassData", [_(massData)]);
  GetNext() => jsvalue.callMethod(r"GetNext", []);
  GetRestitution() => jsvalue.callMethod(r"GetRestitution", []);
  GetShape() => jsvalue.callMethod(r"GetShape", []);
  GetType() => jsvalue.callMethod(r"GetType", []);
  GetUserData() => jsvalue.callMethod(r"GetUserData", []);
  IsSensor() => jsvalue.callMethod(r"IsSensor", []);
  RayCast(output, input) => jsvalue.callMethod(r"RayCast", [_(output), _(input)]);
  SetDensity(density) => jsvalue.callMethod(r"SetDensity", [_(density)]);
  SetFilterData(filter) => jsvalue.callMethod(r"SetFilterData", [_(filter)]);
  SetFriction(friction) => jsvalue.callMethod(r"SetFriction", [_(friction)]);
  SetRestitution(restitution) => jsvalue.callMethod(r"SetRestitution", [_(restitution)]);
  SetSensor(sensor) => jsvalue.callMethod(r"SetSensor", [_(sensor)]);
  SetUserData(data) => jsvalue.callMethod(r"SetUserData", [_(data)]);
  Synchronize(broadPhase, transform1, transform2) => jsvalue.callMethod(r"Synchronize", [_(broadPhase), _(transform1), _(transform2)]);
  TestPoint(p) => jsvalue.callMethod(r"TestPoint", [_(p)]);
  b2Fixture() => jsvalue.callMethod(r"b2Fixture", []);
  static b2Fixture() => _constructor.callMethod(r"b2Fixture", []);
}

class _b2FixtureDef extends _Proxy implements b2FixtureDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2FixtureDef"];
  _b2FixtureDef() : jsvalue = new JsObject(_constructor, []);

  b2FixtureDef() => jsvalue.callMethod(r"b2FixtureDef", []);
  static b2FixtureDef() => _constructor.callMethod(r"b2FixtureDef", []);
}

class _b2Island extends _Proxy implements b2Island {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2Island"];
  _b2Island() : jsvalue = new JsObject(_constructor, []);

  AddBody(body) => jsvalue.callMethod(r"AddBody", [_(body)]);
  AddContact(contact) => jsvalue.callMethod(r"AddContact", [_(contact)]);
  AddJoint(joint) => jsvalue.callMethod(r"AddJoint", [_(joint)]);
  Clear() => jsvalue.callMethod(r"Clear", []);
  Initialize(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) => jsvalue.callMethod(r"Initialize", [_(bodyCapacity), _(contactCapacity), _(jointCapacity), _(allocator), _(listener), _(contactSolver)]);
  Report(constraints) => jsvalue.callMethod(r"Report", [_(constraints)]);
  Solve(step, gravity, allowSleep) => jsvalue.callMethod(r"Solve", [_(step), _(gravity), _(allowSleep)]);
  SolveTOI(subStep) => jsvalue.callMethod(r"SolveTOI", [_(subStep)]);
  b2Island() => jsvalue.callMethod(r"b2Island", []);
  static b2Island() => _constructor.callMethod(r"b2Island", []);
}

class _b2World extends _Proxy implements b2World {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["b2World"];
  _b2World(gravity, doSleep) : jsvalue = new JsObject(_constructor, [_(gravity), _(doSleep)]);

  AddController(c) => jsvalue.callMethod(r"AddController", [_(c)]);
  ClearForces() => jsvalue.callMethod(r"ClearForces", []);
  CreateBody(def) => jsvalue.callMethod(r"CreateBody", [_(def)]);
  CreateController(controller) => jsvalue.callMethod(r"CreateController", [_(controller)]);
  CreateJoint(def) => jsvalue.callMethod(r"CreateJoint", [_(def)]);
  DestroyBody(b) => jsvalue.callMethod(r"DestroyBody", [_(b)]);
  DestroyController(controller) => jsvalue.callMethod(r"DestroyController", [_(controller)]);
  DestroyJoint(j) => jsvalue.callMethod(r"DestroyJoint", [_(j)]);
  DrawDebugData() => jsvalue.callMethod(r"DrawDebugData", []);
  DrawJoint(joint) => jsvalue.callMethod(r"DrawJoint", [_(joint)]);
  DrawShape(shape, xf, color) => jsvalue.callMethod(r"DrawShape", [_(shape), _(xf), _(color)]);
  GetBodyCount() => jsvalue.callMethod(r"GetBodyCount", []);
  GetBodyList() => jsvalue.callMethod(r"GetBodyList", []);
  GetContactCount() => jsvalue.callMethod(r"GetContactCount", []);
  GetContactList() => jsvalue.callMethod(r"GetContactList", []);
  GetGravity() => jsvalue.callMethod(r"GetGravity", []);
  GetGroundBody() => jsvalue.callMethod(r"GetGroundBody", []);
  GetJointCount() => jsvalue.callMethod(r"GetJointCount", []);
  GetJointList() => jsvalue.callMethod(r"GetJointList", []);
  GetProxyCount() => jsvalue.callMethod(r"GetProxyCount", []);
  IsLocked() => jsvalue.callMethod(r"IsLocked", []);
  QueryAABB(callback, aabb) => jsvalue.callMethod(r"QueryAABB", [_(callback), _(aabb)]);
  QueryPoint(callback, p) => jsvalue.callMethod(r"QueryPoint", [_(callback), _(p)]);
  QueryShape(callback, shape, transform) => jsvalue.callMethod(r"QueryShape", [_(callback), _(shape), _(transform)]);
  RayCast(callback, point1, point2) => jsvalue.callMethod(r"RayCast", [_(callback), _(point1), _(point2)]);
  RayCastAll(point1, point2) => jsvalue.callMethod(r"RayCastAll", [_(point1), _(point2)]);
  RayCastOne(point1, point2) => jsvalue.callMethod(r"RayCastOne", [_(point1), _(point2)]);
  RemoveController(c) => jsvalue.callMethod(r"RemoveController", [_(c)]);
  SetBroadPhase(broadPhase) => jsvalue.callMethod(r"SetBroadPhase", [_(broadPhase)]);
  SetContactFilter(filter) => jsvalue.callMethod(r"SetContactFilter", [_(filter)]);
  SetContactListener(listener) => jsvalue.callMethod(r"SetContactListener", [_(listener)]);
  SetContinuousPhysics(flag) => jsvalue.callMethod(r"SetContinuousPhysics", [_(flag)]);
  SetDebugDraw(debugDraw) => jsvalue.callMethod(r"SetDebugDraw", [_(debugDraw)]);
  SetDestructionListener(listener) => jsvalue.callMethod(r"SetDestructionListener", [_(listener)]);
  SetGravity(gravity) => jsvalue.callMethod(r"SetGravity", [_(gravity)]);
  SetWarmStarting(flag) => jsvalue.callMethod(r"SetWarmStarting", [_(flag)]);
  Solve(step) => jsvalue.callMethod(r"Solve", [_(step)]);
  SolveTOI(step) => jsvalue.callMethod(r"SolveTOI", [_(step)]);
  Step(dt, velocityIterations, positionIterations) => jsvalue.callMethod(r"Step", [_(dt), _(velocityIterations), _(positionIterations)]);
  Validate() => jsvalue.callMethod(r"Validate", []);
  b2World(gravity, doSleep) => jsvalue.callMethod(r"b2World", [_(gravity), _(doSleep)]);
  static b2World() => _constructor.callMethod(r"b2World", []);
}

class _b2AABB extends _Proxy implements b2AABB {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2AABB"];
  _b2AABB() : jsvalue = new JsObject(_constructor, []);

  Combine(aabb1, aabb2) => jsvalue.callMethod(r"Combine", [_(aabb1), _(aabb2)]);
  Contains(aabb) => jsvalue.callMethod(r"Contains", [_(aabb)]);
  GetCenter() => jsvalue.callMethod(r"GetCenter", []);
  GetExtents() => jsvalue.callMethod(r"GetExtents", []);
  IsValid() => jsvalue.callMethod(r"IsValid", []);
  RayCast(output, input) => jsvalue.callMethod(r"RayCast", [_(output), _(input)]);
  TestOverlap(other) => jsvalue.callMethod(r"TestOverlap", [_(other)]);
  static Combine(aabb1, aabb2) => _constructor.callMethod(r"Combine", [_(aabb1), _(aabb2)]);
  static b2AABB() => _constructor.callMethod(r"b2AABB", []);
}

class _b2Bound extends _Proxy implements b2Bound {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Bound"];
  _b2Bound() : jsvalue = new JsObject(_constructor, []);

  IsLower() => jsvalue.callMethod(r"IsLower", []);
  IsUpper() => jsvalue.callMethod(r"IsUpper", []);
  Swap(b) => jsvalue.callMethod(r"Swap", [_(b)]);
  static b2Bound() => _constructor.callMethod(r"b2Bound", []);
}

class _b2BoundValues extends _Proxy implements b2BoundValues {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2BoundValues"];
  _b2BoundValues() : jsvalue = new JsObject(_constructor, []);

  b2BoundValues() => jsvalue.callMethod(r"b2BoundValues", []);
  static b2BoundValues() => _constructor.callMethod(r"b2BoundValues", []);
}

class _b2DynamicTree extends _Proxy implements b2DynamicTree {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTree"];
  _b2DynamicTree() : jsvalue = new JsObject(_constructor, []);

  AllocateNode() => jsvalue.callMethod(r"AllocateNode", []);
  CreateProxy(aabb, userData) => jsvalue.callMethod(r"CreateProxy", [_(aabb), _(userData)]);
  DestroyProxy(proxy) => jsvalue.callMethod(r"DestroyProxy", [_(proxy)]);
  FreeNode(node) => jsvalue.callMethod(r"FreeNode", [_(node)]);
  GetFatAABB(proxy) => jsvalue.callMethod(r"GetFatAABB", [_(proxy)]);
  GetUserData(proxy) => jsvalue.callMethod(r"GetUserData", [_(proxy)]);
  InsertLeaf(leaf) => jsvalue.callMethod(r"InsertLeaf", [_(leaf)]);
  MoveProxy(proxy, aabb, displacement) => jsvalue.callMethod(r"MoveProxy", [_(proxy), _(aabb), _(displacement)]);
  Query(callback, aabb) => jsvalue.callMethod(r"Query", [_(callback), _(aabb)]);
  RayCast(callback, input) => jsvalue.callMethod(r"RayCast", [_(callback), _(input)]);
  Rebalance(iterations) => jsvalue.callMethod(r"Rebalance", [_(iterations)]);
  RemoveLeaf(leaf) => jsvalue.callMethod(r"RemoveLeaf", [_(leaf)]);
  b2DynamicTree() => jsvalue.callMethod(r"b2DynamicTree", []);
  static b2DynamicTree() => _constructor.callMethod(r"b2DynamicTree", []);
}

class _b2DynamicTreeBroadPhase extends _Proxy implements b2DynamicTreeBroadPhase {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTreeBroadPhase"];
  _b2DynamicTreeBroadPhase() : jsvalue = new JsObject(_constructor, []);

  BufferMove(proxy) => jsvalue.callMethod(r"BufferMove", [_(proxy)]);
  ComparePairs(pair1, pair2) => jsvalue.callMethod(r"ComparePairs", [_(pair1), _(pair2)]);
  CreateProxy(aabb, userData) => jsvalue.callMethod(r"CreateProxy", [_(aabb), _(userData)]);
  DestroyProxy(proxy) => jsvalue.callMethod(r"DestroyProxy", [_(proxy)]);
  GetFatAABB(proxy) => jsvalue.callMethod(r"GetFatAABB", [_(proxy)]);
  GetProxyCount() => jsvalue.callMethod(r"GetProxyCount", []);
  GetUserData(proxy) => jsvalue.callMethod(r"GetUserData", [_(proxy)]);
  MoveProxy(proxy, aabb, displacement) => jsvalue.callMethod(r"MoveProxy", [_(proxy), _(aabb), _(displacement)]);
  Query(callback, aabb) => jsvalue.callMethod(r"Query", [_(callback), _(aabb)]);
  RayCast(callback, input) => jsvalue.callMethod(r"RayCast", [_(callback), _(input)]);
  Rebalance(iterations) => jsvalue.callMethod(r"Rebalance", [_(iterations)]);
  TestOverlap(proxyA, proxyB) => jsvalue.callMethod(r"TestOverlap", [_(proxyA), _(proxyB)]);
  UnBufferMove(proxy) => jsvalue.callMethod(r"UnBufferMove", [_(proxy)]);
  UpdatePairs(callback) => jsvalue.callMethod(r"UpdatePairs", [_(callback)]);
  Validate() => jsvalue.callMethod(r"Validate", []);
  static b2DynamicTreeBroadPhase() => _constructor.callMethod(r"b2DynamicTreeBroadPhase", []);
}

class _b2DynamicTreeNode extends _Proxy implements b2DynamicTreeNode {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTreeNode"];
  _b2DynamicTreeNode() : jsvalue = new JsObject(_constructor, []);

  IsLeaf() => jsvalue.callMethod(r"IsLeaf", []);
  static b2DynamicTreeNode() => _constructor.callMethod(r"b2DynamicTreeNode", []);
}

class _b2Manifold extends _Proxy implements b2Manifold {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Manifold"];
  _b2Manifold() : jsvalue = new JsObject(_constructor, []);

  Copy() => jsvalue.callMethod(r"Copy", []);
  Reset() => jsvalue.callMethod(r"Reset", []);
  Set(m) => jsvalue.callMethod(r"Set", [_(m)]);
  b2Manifold() => jsvalue.callMethod(r"b2Manifold", []);
  static b2Manifold() => _constructor.callMethod(r"b2Manifold", []);
}

class _b2ManifoldPoint extends _Proxy implements b2ManifoldPoint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2ManifoldPoint"];
  _b2ManifoldPoint() : jsvalue = new JsObject(_constructor, []);

  Reset() => jsvalue.callMethod(r"Reset", []);
  Set(m) => jsvalue.callMethod(r"Set", [_(m)]);
  b2ManifoldPoint() => jsvalue.callMethod(r"b2ManifoldPoint", []);
  static b2ManifoldPoint() => _constructor.callMethod(r"b2ManifoldPoint", []);
}

class _b2Point extends _Proxy implements b2Point {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Point"];
  _b2Point() : jsvalue = new JsObject(_constructor, []);

  GetFirstVertex(xf) => jsvalue.callMethod(r"GetFirstVertex", [_(xf)]);
  Support(xf, vX, vY) => jsvalue.callMethod(r"Support", [_(xf), _(vX), _(vY)]);
  static b2Point() => _constructor.callMethod(r"b2Point", []);
}

class _b2RayCastInput extends _Proxy implements b2RayCastInput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2RayCastInput"];
  _b2RayCastInput(p1, p2, maxFraction) : jsvalue = new JsObject(_constructor, [_(p1), _(p2), _(maxFraction)]);

  b2RayCastInput(p1, p2, maxFraction) => jsvalue.callMethod(r"b2RayCastInput", [_(p1), _(p2), _(maxFraction)]);
  static b2RayCastInput() => _constructor.callMethod(r"b2RayCastInput", []);
}

class _b2Segment extends _Proxy implements b2Segment {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2Segment"];
  _b2Segment() : jsvalue = new JsObject(_constructor, []);

  Extend(aabb) => jsvalue.callMethod(r"Extend", [_(aabb)]);
  ExtendBackward(aabb) => jsvalue.callMethod(r"ExtendBackward", [_(aabb)]);
  ExtendForward(aabb) => jsvalue.callMethod(r"ExtendForward", [_(aabb)]);
  TestSegment(lambda, normal, segment, maxLambda) => jsvalue.callMethod(r"TestSegment", [_(lambda), _(normal), _(segment), _(maxLambda)]);
  static b2Segment() => _constructor.callMethod(r"b2Segment", []);
}

class _b2CircleShape extends _b2Shape implements b2CircleShape {
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2CircleShape"];
  _b2CircleShape(radius) : super.withValue(new JsObject(_constructor, [_(radius)]));

  ComputeAABB(aabb, transform) => jsvalue.callMethod(r"ComputeAABB", [_(aabb), _(transform)]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [_(massData), _(density)]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [_(normal), _(offset), _(xf), _(c)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  GetLocalPosition() => jsvalue.callMethod(r"GetLocalPosition", []);
  GetRadius() => jsvalue.callMethod(r"GetRadius", []);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [_(output), _(input), _(transform)]);
  Set(other) => jsvalue.callMethod(r"Set", [_(other)]);
  SetLocalPosition(position) => jsvalue.callMethod(r"SetLocalPosition", [_(position)]);
  SetRadius(radius) => jsvalue.callMethod(r"SetRadius", [_(radius)]);
  TestPoint(transform, p) => jsvalue.callMethod(r"TestPoint", [_(transform), _(p)]);
  b2CircleShape(radius) => jsvalue.callMethod(r"b2CircleShape", [_(radius)]);
  static b2CircleShape() => _constructor.callMethod(r"b2CircleShape", []);
}

class _b2EdgeChainDef extends _Proxy implements b2EdgeChainDef {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2EdgeChainDef"];
  _b2EdgeChainDef() : jsvalue = new JsObject(_constructor, []);

  b2EdgeChainDef() => jsvalue.callMethod(r"b2EdgeChainDef", []);
  static b2EdgeChainDef() => _constructor.callMethod(r"b2EdgeChainDef", []);
}

class _b2EdgeShape extends _b2Shape implements b2EdgeShape {
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2EdgeShape"];
  _b2EdgeShape(v1, v2) : super.withValue(new JsObject(_constructor, [_(v1), _(v2)]));

  ComputeAABB(aabb, transform) => jsvalue.callMethod(r"ComputeAABB", [_(aabb), _(transform)]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [_(massData), _(density)]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [_(normal), _(offset), _(xf), _(c)]);
  Corner1IsConvex() => jsvalue.callMethod(r"Corner1IsConvex", []);
  Corner2IsConvex() => jsvalue.callMethod(r"Corner2IsConvex", []);
  GetCoreVertex1() => jsvalue.callMethod(r"GetCoreVertex1", []);
  GetCoreVertex2() => jsvalue.callMethod(r"GetCoreVertex2", []);
  GetCorner1Vector() => jsvalue.callMethod(r"GetCorner1Vector", []);
  GetCorner2Vector() => jsvalue.callMethod(r"GetCorner2Vector", []);
  GetDirectionVector() => jsvalue.callMethod(r"GetDirectionVector", []);
  GetFirstVertex(xf) => jsvalue.callMethod(r"GetFirstVertex", [_(xf)]);
  GetLength() => jsvalue.callMethod(r"GetLength", []);
  GetNextEdge() => jsvalue.callMethod(r"GetNextEdge", []);
  GetNormalVector() => jsvalue.callMethod(r"GetNormalVector", []);
  GetPrevEdge() => jsvalue.callMethod(r"GetPrevEdge", []);
  GetVertex1() => jsvalue.callMethod(r"GetVertex1", []);
  GetVertex2() => jsvalue.callMethod(r"GetVertex2", []);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [_(output), _(input), _(transform)]);
  SetNextEdge(edge, core, cornerDir, convex) => jsvalue.callMethod(r"SetNextEdge", [_(edge), _(core), _(cornerDir), _(convex)]);
  SetPrevEdge(edge, core, cornerDir, convex) => jsvalue.callMethod(r"SetPrevEdge", [_(edge), _(core), _(cornerDir), _(convex)]);
  Support(xf, dX, dY) => jsvalue.callMethod(r"Support", [_(xf), _(dX), _(dY)]);
  TestPoint(transform, p) => jsvalue.callMethod(r"TestPoint", [_(transform), _(p)]);
  b2EdgeShape(v1, v2) => jsvalue.callMethod(r"b2EdgeShape", [_(v1), _(v2)]);
  static b2EdgeShape() => _constructor.callMethod(r"b2EdgeShape", []);
}

class _b2PolygonShape extends _b2Shape implements b2PolygonShape {
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2PolygonShape"];
  _b2PolygonShape() : super.withValue(new JsObject(_constructor, []));

  ComputeAABB(aabb, xf) => jsvalue.callMethod(r"ComputeAABB", [_(aabb), _(xf)]);
  ComputeMass(massData, density) => jsvalue.callMethod(r"ComputeMass", [_(massData), _(density)]);
  ComputeSubmergedArea(normal, offset, xf, c) => jsvalue.callMethod(r"ComputeSubmergedArea", [_(normal), _(offset), _(xf), _(c)]);
  Copy() => jsvalue.callMethod(r"Copy", []);
  GetNormals() => jsvalue.callMethod(r"GetNormals", []);
  GetSupport(d) => jsvalue.callMethod(r"GetSupport", [_(d)]);
  GetSupportVertex(d) => jsvalue.callMethod(r"GetSupportVertex", [_(d)]);
  GetVertexCount() => jsvalue.callMethod(r"GetVertexCount", []);
  GetVertices() => jsvalue.callMethod(r"GetVertices", []);
  RayCast(output, input, transform) => jsvalue.callMethod(r"RayCast", [_(output), _(input), _(transform)]);
  Reserve(count) => jsvalue.callMethod(r"Reserve", [_(count)]);
  Set(other) => jsvalue.callMethod(r"Set", [_(other)]);
  SetAsArray(vertices, vertexCount) => jsvalue.callMethod(r"SetAsArray", [_(vertices), _(vertexCount)]);
  SetAsBox(hx, hy) => jsvalue.callMethod(r"SetAsBox", [_(hx), _(hy)]);
  SetAsEdge(v1, v2) => jsvalue.callMethod(r"SetAsEdge", [_(v1), _(v2)]);
  SetAsOrientedBox(hx, hy, center, angle) => jsvalue.callMethod(r"SetAsOrientedBox", [_(hx), _(hy), _(center), _(angle)]);
  SetAsVector(vertices, vertexCount) => jsvalue.callMethod(r"SetAsVector", [_(vertices), _(vertexCount)]);
  TestPoint(xf, p) => jsvalue.callMethod(r"TestPoint", [_(xf), _(p)]);
  Validate() => jsvalue.callMethod(r"Validate", []);
  b2PolygonShape() => jsvalue.callMethod(r"b2PolygonShape", []);
  static AsArray(vertices, vertexCount) => _constructor.callMethod(r"AsArray", [_(vertices), _(vertexCount)]);
  static AsBox(hx, hy) => _constructor.callMethod(r"AsBox", [_(hx), _(hy)]);
  static AsEdge(v1, v2) => _constructor.callMethod(r"AsEdge", [_(v1), _(v2)]);
  static AsOrientedBox(hx, hy, center, angle) => _constructor.callMethod(r"AsOrientedBox", [_(hx), _(hy), _(center), _(angle)]);
  static AsVector(vertices, vertexCount) => _constructor.callMethod(r"AsVector", [_(vertices), _(vertexCount)]);
  static ComputeCentroid(vs, count) => _constructor.callMethod(r"ComputeCentroid", [_(vs), _(count)]);
  static ComputeOBB(obb, vs, count) => _constructor.callMethod(r"ComputeOBB", [_(obb), _(vs), _(count)]);
  static b2PolygonShape() => _constructor.callMethod(r"b2PolygonShape", []);
}

class _b2CircleContact extends _b2Contact implements b2CircleContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2CircleContact"];
  _b2CircleContact() : super.withValue(new JsObject(_constructor, []));

  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [_(fixtureA), _(fixtureB)]);
  static Create(allocator) => _constructor.callMethod(r"Create", [_(allocator)]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [_(contact), _(allocator)]);
  static b2CircleContact() => _constructor.callMethod(r"b2CircleContact", []);
}

class _b2ContactConstraint extends _Proxy implements b2ContactConstraint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactConstraint"];
  _b2ContactConstraint() : jsvalue = new JsObject(_constructor, []);

  b2ContactConstraint() => jsvalue.callMethod(r"b2ContactConstraint", []);
  static b2ContactConstraint() => _constructor.callMethod(r"b2ContactConstraint", []);
}

class _b2ContactFactory extends _Proxy implements b2ContactFactory {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactFactory"];
  _b2ContactFactory(allocator) : jsvalue = new JsObject(_constructor, [_(allocator)]);

  AddType(createFcn, destroyFcn, type1, type2) => jsvalue.callMethod(r"AddType", [_(createFcn), _(destroyFcn), _(type1), _(type2)]);
  Create(fixtureA, fixtureB) => jsvalue.callMethod(r"Create", [_(fixtureA), _(fixtureB)]);
  Destroy(contact) => jsvalue.callMethod(r"Destroy", [_(contact)]);
  InitializeRegisters() => jsvalue.callMethod(r"InitializeRegisters", []);
  b2ContactFactory(allocator) => jsvalue.callMethod(r"b2ContactFactory", [_(allocator)]);
  static b2ContactFactory() => _constructor.callMethod(r"b2ContactFactory", []);
}

class _b2ContactSolver extends _Proxy implements b2ContactSolver {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactSolver"];
  _b2ContactSolver() : jsvalue = new JsObject(_constructor, []);

  FinalizeVelocityConstraints() => jsvalue.callMethod(r"FinalizeVelocityConstraints", []);
  InitVelocityConstraints(step) => jsvalue.callMethod(r"InitVelocityConstraints", [_(step)]);
  Initialize(step, contacts, contactCount, allocator) => jsvalue.callMethod(r"Initialize", [_(step), _(contacts), _(contactCount), _(allocator)]);
  SolvePositionConstraints(baumgarte) => jsvalue.callMethod(r"SolvePositionConstraints", [_(baumgarte)]);
  SolveVelocityConstraints() => jsvalue.callMethod(r"SolveVelocityConstraints", []);
  b2ContactSolver() => jsvalue.callMethod(r"b2ContactSolver", []);
  static b2ContactSolver() => _constructor.callMethod(r"b2ContactSolver", []);
}

class _b2EdgeAndCircleContact extends _b2Contact implements b2EdgeAndCircleContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2EdgeAndCircleContact"];
  _b2EdgeAndCircleContact() : super.withValue(new JsObject(_constructor, []));

  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [_(fixtureA), _(fixtureB)]);
  b2CollideEdgeAndCircle(manifold, edge, xf1, circle, xf2) => jsvalue.callMethod(r"b2CollideEdgeAndCircle", [_(manifold), _(edge), _(xf1), _(circle), _(xf2)]);
  static Create(allocator) => _constructor.callMethod(r"Create", [_(allocator)]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [_(contact), _(allocator)]);
  static b2EdgeAndCircleContact() => _constructor.callMethod(r"b2EdgeAndCircleContact", []);
}

class _b2NullContact extends _b2Contact implements b2NullContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2NullContact"];
  _b2NullContact() : super.withValue(new JsObject(_constructor, []));

  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  b2NullContact() => jsvalue.callMethod(r"b2NullContact", []);
  static b2NullContact() => _constructor.callMethod(r"b2NullContact", []);
}

class _b2PolyAndCircleContact extends _b2Contact implements b2PolyAndCircleContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PolyAndCircleContact"];
  _b2PolyAndCircleContact() : super.withValue(new JsObject(_constructor, []));

  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [_(fixtureA), _(fixtureB)]);
  static Create(allocator) => _constructor.callMethod(r"Create", [_(allocator)]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [_(contact), _(allocator)]);
  static b2PolyAndCircleContact() => _constructor.callMethod(r"b2PolyAndCircleContact", []);
}

class _b2PolyAndEdgeContact extends _b2Contact implements b2PolyAndEdgeContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PolyAndEdgeContact"];
  _b2PolyAndEdgeContact() : super.withValue(new JsObject(_constructor, []));

  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [_(fixtureA), _(fixtureB)]);
  b2CollidePolyAndEdge(manifold, polygon, xf1, edge, xf2) => jsvalue.callMethod(r"b2CollidePolyAndEdge", [_(manifold), _(polygon), _(xf1), _(edge), _(xf2)]);
  static Create(allocator) => _constructor.callMethod(r"Create", [_(allocator)]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [_(contact), _(allocator)]);
  static b2PolyAndEdgeContact() => _constructor.callMethod(r"b2PolyAndEdgeContact", []);
}

class _b2PolygonContact extends _b2Contact implements b2PolygonContact {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2PolygonContact"];
  _b2PolygonContact() : super.withValue(new JsObject(_constructor, []));

  Evaluate() => jsvalue.callMethod(r"Evaluate", []);
  Reset(fixtureA, fixtureB) => jsvalue.callMethod(r"Reset", [_(fixtureA), _(fixtureB)]);
  static Create(allocator) => _constructor.callMethod(r"Create", [_(allocator)]);
  static Destroy(contact, allocator) => _constructor.callMethod(r"Destroy", [_(contact), _(allocator)]);
  static b2PolygonContact() => _constructor.callMethod(r"b2PolygonContact", []);
}

class _b2BuoyancyController extends _b2Controller implements b2BuoyancyController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2BuoyancyController"];
  _b2BuoyancyController() : super.withValue(new JsObject(_constructor, []));

  Draw(debugDraw) => jsvalue.callMethod(r"Draw", [_(debugDraw)]);
  Step(step) => jsvalue.callMethod(r"Step", [_(step)]);
  static b2BuoyancyController() => _constructor.callMethod(r"b2BuoyancyController", []);
}

class _b2ConstantAccelController extends _b2Controller implements b2ConstantAccelController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2ConstantAccelController"];
  _b2ConstantAccelController() : super.withValue(new JsObject(_constructor, []));

  Step(step) => jsvalue.callMethod(r"Step", [_(step)]);
  static b2ConstantAccelController() => _constructor.callMethod(r"b2ConstantAccelController", []);
}

class _b2ConstantForceController extends _b2Controller implements b2ConstantForceController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2ConstantForceController"];
  _b2ConstantForceController() : super.withValue(new JsObject(_constructor, []));

  Step(step) => jsvalue.callMethod(r"Step", [_(step)]);
  static b2ConstantForceController() => _constructor.callMethod(r"b2ConstantForceController", []);
}

class _b2GravityController extends _b2Controller implements b2GravityController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2GravityController"];
  _b2GravityController() : super.withValue(new JsObject(_constructor, []));

  Step(step) => jsvalue.callMethod(r"Step", [_(step)]);
  static b2GravityController() => _constructor.callMethod(r"b2GravityController", []);
}

class _b2TensorDampingController extends _b2Controller implements b2TensorDampingController {
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2TensorDampingController"];
  _b2TensorDampingController() : super.withValue(new JsObject(_constructor, []));

  SetAxisAligned(xDamping, yDamping) => jsvalue.callMethod(r"SetAxisAligned", [_(xDamping), _(yDamping)]);
  Step(step) => jsvalue.callMethod(r"Step", [_(step)]);
  static b2TensorDampingController() => _constructor.callMethod(r"b2TensorDampingController", []);
}

class _b2ControllerEdge extends _Proxy implements b2ControllerEdge {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Controllers"]["b2ControllerEdge"];
  _b2ControllerEdge() : jsvalue = new JsObject(_constructor, []);

  static b2ControllerEdge() => _constructor.callMethod(r"b2ControllerEdge", []);
}

class _Vector_a2j_Number extends _Proxy implements Vector_a2j_Number {
  var jsvalue;
  static JsObject _constructor = context["Vector_a2j_Number"];
  _Vector_a2j_Number(length) : jsvalue = new JsObject(_constructor, [_(length)]);

}

class _b2ContactRegister extends _Proxy implements b2ContactRegister {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactRegister"];
  _b2ContactRegister() : jsvalue = new JsObject(_constructor, []);

  static b2ContactRegister() => _constructor.callMethod(r"b2ContactRegister", []);
}

class _b2ContactConstraintPoint extends _Proxy implements b2ContactConstraintPoint {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactConstraintPoint"];
  _b2ContactConstraintPoint() : jsvalue = new JsObject(_constructor, []);

  static b2ContactConstraintPoint() => _constructor.callMethod(r"b2ContactConstraintPoint", []);
}

class _b2ContactEdge extends _Proxy implements b2ContactEdge {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Contacts"]["b2ContactEdge"];
  _b2ContactEdge() : jsvalue = new JsObject(_constructor, []);

  static b2ContactEdge() => _constructor.callMethod(r"b2ContactEdge", []);
}

class _b2MassData extends _Proxy implements b2MassData {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["Shapes"]["b2MassData"];
  _b2MassData() : jsvalue = new JsObject(_constructor, []);

  static b2MassData() => _constructor.callMethod(r"b2MassData", []);
}

class _b2DynamicTreePair extends _Proxy implements b2DynamicTreePair {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2DynamicTreePair"];
  _b2DynamicTreePair() : jsvalue = new JsObject(_constructor, []);

  static b2DynamicTreePair() => _constructor.callMethod(r"b2DynamicTreePair", []);
}

class _b2RayCastOutput extends _Proxy implements b2RayCastOutput {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Collision"]["b2RayCastOutput"];
  _b2RayCastOutput() : jsvalue = new JsObject(_constructor, []);

  static b2RayCastOutput() => _constructor.callMethod(r"b2RayCastOutput", []);
}

class _b2JointEdge extends _Proxy implements b2JointEdge {
  var jsvalue;
  static JsObject _constructor = context["Box2D"]["Dynamics"]["Joints"]["b2JointEdge"];
  _b2JointEdge() : jsvalue = new JsObject(_constructor, []);

  static b2JointEdge() => _constructor.callMethod(r"b2JointEdge", []);
}

