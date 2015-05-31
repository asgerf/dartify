library fabricjs;

import 'dart:js';

abstract class Object$ {
  factory Object$(options) = _Object;

  isActive();
  setActive(active);
  animate(property, to, options);
  getBoundingRectHeight();
  getBoundingRectWidth();
  bringForward();
  bringToFront();
  isContainedWithinObject(other);
  isContainedWithinRect(selectionTL, selectionBR);
  setCoords();
  callSuper(List args);
  center();
  centerH();
  centerV();
  clone(callback, propertiesToInclude);
  cloneAsImage(callback);
  complexity();
  drawBorders(ctx);
  drawCorners(ctx);
  fire(eventName, options);
  fxStraighten(callbacks);
  setGradientFill(options);
  get(property);
  getHeight();
  hasStateChanged();
  intersectsWithObject(other);
  intersectsWithRect(selectionTL, selectionBR);
  setOptions(options);
  observe(List args);
  off(eventName, handler);
  on(List args);
  remove();
  render(ctx, noTransform);
  setSourcePath(value);
  getSvgStyles();
  getSvgTransform();
  saveState();
  scale(value);
  scaleToHeight(value);
  scaleToWidth(value);
  sendBackwards();
  sendToBack();
  set(key, value);
  setupState();
  stopObserving(eventName, handler);
  straighten();
  isType(type);
  toDataURL(callback);
  toDatalessObject(propertiesToInclude);
  toGrayscale();
  toJSON(propertiesToInclude);
  toObject(propertiesToInclude);
  toString();
  toggle(property);
  transform(ctx);
  getWidth();
}

abstract class Path extends Object$ {
  factory Path(path, options) = _Path;

  callSuper(List args);
  complexity();
  render(ctx, noTransform);
  toDatalessObject(propertiesToInclude);
  toObject(propertiesToInclude);
  toSVG();
  toString();

  static fromElement(element, options) => _Path.fromElement(element, options);
  static fromObject(object) => _Path.fromObject(object);
}

abstract class StaticCanvas {
  factory StaticCanvas(el, options) = _StaticCanvas;

  getActiveGroup();
  getActiveObject();
  add(List args);
  setBackgroundImage(url, callback, options);
  bringForward(object);
  bringToFront(object);
  getCenter();
  getContext();
  calcOffset();
  centerObject(object);
  centerObjectH(object);
  centerObjectV(object);
  clear();
  clearContext(ctx);
  clone(callback);
  cloneWithoutData(callback);
  complexity();
  setDimensions(dimensions);
  dispose();
  drawControls(ctx);
  getElement();
  isEmpty();
  fire(eventName, options);
  forEachObject(callback, context);
  fxCenterObjectH(object, callbacks);
  fxCenterObjectV(object, callbacks);
  fxRemove(object, callbacks);
  fxStraightenObject(object);
  getHeight();
  setHeight(value);
  insertAt(object, index, nonSplicing);
  item(index);
  loadFromDatalessJSON(json, callback);
  loadFromJSON(json, callback);
  getObjects();
  setOverlayImage(url, callback, options);
  observe(List args);
  off(eventName, handler);
  on(List args);
  onBeforeScaleRotate();
  remove(object);
  renderAll(allOnTop);
  renderTop();
  sendBackwards(object);
  sendToBack(object);
  stopObserving(eventName, handler);
  straightenObject(object);
  toDataURL(format, quality);
  toDataURLWithMultiplier(format, multiplier, quality);
  toDatalessJSON(propertiesToInclude);
  toDatalessObject(propertiesToInclude);
  toJSON(propertiesToInclude);
  toObject(propertiesToInclude);
  toSVG();
  toString();
  getWidth();
  setWidth(value);

  static supports(methodName) => _StaticCanvas.supports(methodName);
  static toGrayscale(canvasEl) => _StaticCanvas.toGrayscale(canvasEl);
}

abstract class Gradient {
  factory Gradient(options) = _Gradient;

  callSuper(List args);
  toLiveGradient(ctx);
  toObject();

  static forObject(obj, options) => _Gradient.forObject(obj, options);
  static fromElement(el, instance) => _Gradient.fromElement(el, instance);
}

abstract class Point {
  factory Point(List args) = _Point;

  add(that);
  addEquals(that);
  distanceFrom(that);
  divide(scalar);
  divideEquals(scalar);
  eq(that);
  setFromPoint(that);
  gt(that);
  gte(that);
  init(x, y);
  lerp(that, t);
  lt(that);
  lte(that);
  max(that);
  midPointFrom(that);
  min(that);
  multiply(scalar);
  multiplyEquals(scalar);
  scalarAdd(scalar);
  scalarAddEquals(scalar);
  scalarSubtract(scalar);
  scalarSubtractEquals(scalar);
  subtract(that);
  subtractEquals(that);
  swap(that);
  toString();
  setXY(x, y);
}

abstract class Intersection {
  factory Intersection(List args) = _Intersection;

  appendPoint(point);
  appendPoints(points);
  init(status);

  static intersectLineLine(a1, a2, b1, b2) => _Intersection.intersectLineLine(a1, a2, b1, b2);
  static intersectLinePolygon(a1, a2, points) => _Intersection.intersectLinePolygon(a1, a2, points);
  static intersectPolygonPolygon(points1, points2) => _Intersection.intersectPolygonPolygon(points1, points2);
  static intersectPolygonRectangle(points, r1, r2) => _Intersection.intersectPolygonRectangle(points, r1, r2);
}

abstract class Color {
  factory Color(color) = _Color;

  getAlpha();
  setAlpha(alpha);
  overlayWith(otherColor);
  getSource();
  setSource(source);
  toBlackWhite(threshold);
  toGrayscale();
  toHex();
  toRgb();
  toRgba();

  static fromHex(color) => _Color.fromHex(color);
  static fromRgb(color) => _Color.fromRgb(color);
  static fromRgba(color) => _Color.fromRgba(color);
  static fromSource(source) => _Color.fromSource(source);
  static sourceFromHex(color) => _Color.sourceFromHex(color);
  static sourceFromRgb(color) => _Color.sourceFromRgb(color);
}

abstract class FreeDrawing {
  factory FreeDrawing(fabricCanvas) = _FreeDrawing;

  callSuper(List args);
  convertPointsToSVGPath(points, minX, maxX, minY, maxY);
  getPathBoundingBox(points);
}

abstract class Canvas extends StaticCanvas {
  factory Canvas(el, options) = _Canvas;

  getActiveGroup();
  setActiveGroup(group);
  getActiveObject();
  setActiveObject(object, e);
  containsPoint(e, target);
  deactivateAll();
  deactivateAllWithDispatch();
  discardActiveGroup();
  discardActiveObject();
  drawDashedLine(ctx, x, y, x2, y2, da);
  findTarget(e, skipGroup);
  getPointer(e);
  getSelectionContext();
  getSelectionElement();
  toString();

  static bind(List args) => _Canvas.bind(args);
  static supports(methodName) => _Canvas.supports(methodName);
  static toGrayscale(canvasEl) => _Canvas.toGrayscale(canvasEl);
}

abstract class Line extends Object$ {
  factory Line(points, options) = _Line;

  callSuper(List args);
  complexity();
  toObject(propertiesToInclude);
  toSVG();

  static fromElement(element, options) => _Line.fromElement(element, options);
  static fromObject(object) => _Line.fromObject(object);
}

abstract class Circle extends Object$ {
  factory Circle(options) = _Circle;

  callSuper(List args);
  complexity();
  getRadiusX();
  getRadiusY();
  setRadius(value);
  toObject(propertiesToInclude);
  toSVG();

  static fromElement(element, options) => _Circle.fromElement(element, options);
  static fromObject(object) => _Circle.fromObject(object);
}

abstract class Triangle extends Object$ {
  factory Triangle(options) = _Triangle;

  callSuper(List args);
  complexity();
  toSVG();

  static fromObject(object) => _Triangle.fromObject(object);
}

abstract class Ellipse extends Object$ {
  factory Ellipse(options) = _Ellipse;

  callSuper(List args);
  complexity();
  render(ctx, noTransform);
  toObject(propertiesToInclude);
  toSVG();

  static fromElement(element, options) => _Ellipse.fromElement(element, options);
  static fromObject(object) => _Ellipse.fromObject(object);
}

abstract class Rect extends Object$ {
  factory Rect(options) = _Rect;

  callSuper(List args);
  complexity();
  toObject(propertiesToInclude);
  toSVG();

  static fromElement(element, options) => _Rect.fromElement(element, options);
  static fromObject(object) => _Rect.fromObject(object);
}

abstract class Polyline extends Object$ {
  factory Polyline(points, options) = _Polyline;

  callSuper(List args);
  complexity();
  toObject(propertiesToInclude);
  toSVG();

  static fromElement(element, options) => _Polyline.fromElement(element, options);
  static fromObject(object) => _Polyline.fromObject(object);
}

abstract class Polygon extends Object$ {
  factory Polygon(points, options) = _Polygon;

  callSuper(List args);
  complexity();
  toObject(propertiesToInclude);
  toSVG();

  static fromElement(element, options) => _Polygon.fromElement(element, options);
  static fromObject(object) => _Polygon.fromObject(object);
}

abstract class PathGroup extends Path {
  factory PathGroup(paths, options) = _PathGroup;

  callSuper(List args);
  complexity();
  getObjects();
  render(ctx);
  isSameColor();
  toDatalessObject(propertiesToInclude);
  toGrayscale();
  toObject(propertiesToInclude);
  toSVG();
  toString();

  static fromObject(object) => _PathGroup.fromObject(object);
}

abstract class Group extends Object$ {
  factory Group(objects, options) = _Group;

  activateAllObjects();
  add(object);
  addWithUpdate(object);
  callSuper(List args);
  complexity();
  contains(object);
  containsPoint(point);
  destroy();
  forEachObject(callback, context);
  get(prop);
  hasMoved();
  item(index);
  setObjectsCoords();
  getObjects();
  remove(object);
  removeWithUpdate(object);
  render(ctx, noTransform);
  saveCoords();
  size();
  toGrayscale();
  toObject(propertiesToInclude);
  toSVG();
  toString();

  static fromObject(object, callback) => _Group.fromObject(object, callback);
}

abstract class Image extends Object$ {
  factory Image(element, options) = _Image;

  applyFilters(callback);
  callSuper(List args);
  clone(callback, propertiesToInclude);
  complexity();
  getElement();
  setElement(element);
  getOriginalSize();
  render(ctx, noTransform);
  getSrc();
  getSvgSrc();
  toObject(propertiesToInclude);
  toSVG();
  toString();

  static fromElement(element, callback, options) => _Image.fromElement(element, callback, options);
  static fromObject(object, callback) => _Image.fromObject(object, callback);
  static fromURL(url, callback, imgOptions) => _Image.fromURL(url, callback, imgOptions);
}

abstract class Text extends Object$ {
  factory Text(text, options) = _Text;

  setColor(value);
  callSuper(List args);
  setFontsize(value);
  render(ctx, noTransform);
  getText();
  setText(value);
  toObject(propertiesToInclude);
  toSVG();
  toString();

  static fromElement(element, options) => _Text.fromElement(element, options);
  static fromObject(object) => _Text.fromObject(object);
}

abstract class Grayscale {
  factory Grayscale() = _Grayscale;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject() => _Grayscale.fromObject();
}

abstract class RemoveWhite {
  factory RemoveWhite(options) = _RemoveWhite;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject(object) => _RemoveWhite.fromObject(object);
}

abstract class Invert {
  factory Invert() = _Invert;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject() => _Invert.fromObject();
}

abstract class Sepia {
  factory Sepia() = _Sepia;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject() => _Sepia.fromObject();
}

abstract class Sepia2 {
  factory Sepia2() = _Sepia2;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject() => _Sepia2.fromObject();
}

abstract class Brightness {
  factory Brightness(options) = _Brightness;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject(object) => _Brightness.fromObject(object);
}

abstract class Noise {
  factory Noise(options) = _Noise;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject(object) => _Noise.fromObject(object);
}

abstract class GradientTransparency {
  factory GradientTransparency(options) = _GradientTransparency;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject(object) => _GradientTransparency.fromObject(object);
}

abstract class Tint {
  factory Tint(options) = _Tint;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject(object) => _Tint.fromObject(object);
}

abstract class Convolute {
  factory Convolute(options) = _Convolute;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject(object) => _Convolute.fromObject(object);
}

abstract class Pixelate {
  factory Pixelate(options) = _Pixelate;

  applyTo(canvasEl);
  callSuper(List args);
  toJSON();

  static fromObject(object) => _Pixelate.fromObject(object);
}

abstract class request {
  factory request(url, options) = _request;

}

abstract class Size {
  factory Size(value, base) = _Size;

  var value;
  var unit;
  var convert;
  var convertFrom;
}

  abstract class _Proxy {
    get jsvalue;
  }

  _(x) {
    if (x is _Proxy) return x.jsvalue;
    if (x is Map || x is Iterable) return new JsObject.jsify(x);
    return x;
  }

class _Object extends _Proxy implements Object$ {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Object"];
  _Object(options) : jsvalue = new JsObject(_constructor, [_(options)]);
  _Object.withValue(value) : jsvalue = value;

  isActive() => jsvalue.callMethod(r"isActive", []);
  setActive(active) => jsvalue.callMethod(r"setActive", [_(active)]);
  animate(property, to, options) => jsvalue.callMethod(r"animate", [_(property), _(to), _(options)]);
  getBoundingRectHeight() => jsvalue.callMethod(r"getBoundingRectHeight", []);
  getBoundingRectWidth() => jsvalue.callMethod(r"getBoundingRectWidth", []);
  bringForward() => jsvalue.callMethod(r"bringForward", []);
  bringToFront() => jsvalue.callMethod(r"bringToFront", []);
  isContainedWithinObject(other) => jsvalue.callMethod(r"isContainedWithinObject", [_(other)]);
  isContainedWithinRect(selectionTL, selectionBR) => jsvalue.callMethod(r"isContainedWithinRect", [_(selectionTL), _(selectionBR)]);
  setCoords() => jsvalue.callMethod(r"setCoords", []);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  center() => jsvalue.callMethod(r"center", []);
  centerH() => jsvalue.callMethod(r"centerH", []);
  centerV() => jsvalue.callMethod(r"centerV", []);
  clone(callback, propertiesToInclude) => jsvalue.callMethod(r"clone", [_(callback), _(propertiesToInclude)]);
  cloneAsImage(callback) => jsvalue.callMethod(r"cloneAsImage", [_(callback)]);
  complexity() => jsvalue.callMethod(r"complexity", []);
  drawBorders(ctx) => jsvalue.callMethod(r"drawBorders", [_(ctx)]);
  drawCorners(ctx) => jsvalue.callMethod(r"drawCorners", [_(ctx)]);
  fire(eventName, options) => jsvalue.callMethod(r"fire", [_(eventName), _(options)]);
  fxStraighten(callbacks) => jsvalue.callMethod(r"fxStraighten", [_(callbacks)]);
  setGradientFill(options) => jsvalue.callMethod(r"setGradientFill", [_(options)]);
  get(property) => jsvalue.callMethod(r"get", [_(property)]);
  getHeight() => jsvalue.callMethod(r"getHeight", []);
  hasStateChanged() => jsvalue.callMethod(r"hasStateChanged", []);
  intersectsWithObject(other) => jsvalue.callMethod(r"intersectsWithObject", [_(other)]);
  intersectsWithRect(selectionTL, selectionBR) => jsvalue.callMethod(r"intersectsWithRect", [_(selectionTL), _(selectionBR)]);
  setOptions(options) => jsvalue.callMethod(r"setOptions", [_(options)]);
  observe(args) => jsvalue.callMethod(r"observe", args.map(_).toList());
  off(eventName, handler) => jsvalue.callMethod(r"off", [_(eventName), _(handler)]);
  on(args) => jsvalue.callMethod(r"on", args.map(_).toList());
  remove() => jsvalue.callMethod(r"remove", []);
  render(ctx, noTransform) => jsvalue.callMethod(r"render", [_(ctx), _(noTransform)]);
  setSourcePath(value) => jsvalue.callMethod(r"setSourcePath", [_(value)]);
  getSvgStyles() => jsvalue.callMethod(r"getSvgStyles", []);
  getSvgTransform() => jsvalue.callMethod(r"getSvgTransform", []);
  saveState() => jsvalue.callMethod(r"saveState", []);
  scale(value) => jsvalue.callMethod(r"scale", [_(value)]);
  scaleToHeight(value) => jsvalue.callMethod(r"scaleToHeight", [_(value)]);
  scaleToWidth(value) => jsvalue.callMethod(r"scaleToWidth", [_(value)]);
  sendBackwards() => jsvalue.callMethod(r"sendBackwards", []);
  sendToBack() => jsvalue.callMethod(r"sendToBack", []);
  set(key, value) => jsvalue.callMethod(r"set", [_(key), _(value)]);
  setupState() => jsvalue.callMethod(r"setupState", []);
  stopObserving(eventName, handler) => jsvalue.callMethod(r"stopObserving", [_(eventName), _(handler)]);
  straighten() => jsvalue.callMethod(r"straighten", []);
  isType(type) => jsvalue.callMethod(r"isType", [_(type)]);
  toDataURL(callback) => jsvalue.callMethod(r"toDataURL", [_(callback)]);
  toDatalessObject(propertiesToInclude) => jsvalue.callMethod(r"toDatalessObject", [_(propertiesToInclude)]);
  toGrayscale() => jsvalue.callMethod(r"toGrayscale", []);
  toJSON(propertiesToInclude) => jsvalue.callMethod(r"toJSON", [_(propertiesToInclude)]);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toString() => jsvalue.callMethod(r"toString", []);
  toggle(property) => jsvalue.callMethod(r"toggle", [_(property)]);
  transform(ctx) => jsvalue.callMethod(r"transform", [_(ctx)]);
  getWidth() => jsvalue.callMethod(r"getWidth", []);
}

class _Path extends _Object implements Path {
  static JsObject _constructor = context["fabric"]["Path"];
  _Path(path, options) : super.withValue(new JsObject(_constructor, [_(path), _(options)]));
  _Path.withValue(value) : super.withValue(value);

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  render(ctx, noTransform) => jsvalue.callMethod(r"render", [_(ctx), _(noTransform)]);
  toDatalessObject(propertiesToInclude) => jsvalue.callMethod(r"toDatalessObject", [_(propertiesToInclude)]);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  toString() => jsvalue.callMethod(r"toString", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _StaticCanvas extends _Proxy implements StaticCanvas {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["StaticCanvas"];
  _StaticCanvas(el, options) : jsvalue = new JsObject(_constructor, [_(el), _(options)]);
  _StaticCanvas.withValue(value) : jsvalue = value;

  getActiveGroup() => jsvalue.callMethod(r"getActiveGroup", []);
  getActiveObject() => jsvalue.callMethod(r"getActiveObject", []);
  add(args) => jsvalue.callMethod(r"add", args.map(_).toList());
  setBackgroundImage(url, callback, options) => jsvalue.callMethod(r"setBackgroundImage", [_(url), _(callback), _(options)]);
  bringForward(object) => jsvalue.callMethod(r"bringForward", [_(object)]);
  bringToFront(object) => jsvalue.callMethod(r"bringToFront", [_(object)]);
  getCenter() => jsvalue.callMethod(r"getCenter", []);
  getContext() => jsvalue.callMethod(r"getContext", []);
  calcOffset() => jsvalue.callMethod(r"calcOffset", []);
  centerObject(object) => jsvalue.callMethod(r"centerObject", [_(object)]);
  centerObjectH(object) => jsvalue.callMethod(r"centerObjectH", [_(object)]);
  centerObjectV(object) => jsvalue.callMethod(r"centerObjectV", [_(object)]);
  clear() => jsvalue.callMethod(r"clear", []);
  clearContext(ctx) => jsvalue.callMethod(r"clearContext", [_(ctx)]);
  clone(callback) => jsvalue.callMethod(r"clone", [_(callback)]);
  cloneWithoutData(callback) => jsvalue.callMethod(r"cloneWithoutData", [_(callback)]);
  complexity() => jsvalue.callMethod(r"complexity", []);
  setDimensions(dimensions) => jsvalue.callMethod(r"setDimensions", [_(dimensions)]);
  dispose() => jsvalue.callMethod(r"dispose", []);
  drawControls(ctx) => jsvalue.callMethod(r"drawControls", [_(ctx)]);
  getElement() => jsvalue.callMethod(r"getElement", []);
  isEmpty() => jsvalue.callMethod(r"isEmpty", []);
  fire(eventName, options) => jsvalue.callMethod(r"fire", [_(eventName), _(options)]);
  forEachObject(callback, context) => jsvalue.callMethod(r"forEachObject", [_(callback), _(context)]);
  fxCenterObjectH(object, callbacks) => jsvalue.callMethod(r"fxCenterObjectH", [_(object), _(callbacks)]);
  fxCenterObjectV(object, callbacks) => jsvalue.callMethod(r"fxCenterObjectV", [_(object), _(callbacks)]);
  fxRemove(object, callbacks) => jsvalue.callMethod(r"fxRemove", [_(object), _(callbacks)]);
  fxStraightenObject(object) => jsvalue.callMethod(r"fxStraightenObject", [_(object)]);
  getHeight() => jsvalue.callMethod(r"getHeight", []);
  setHeight(value) => jsvalue.callMethod(r"setHeight", [_(value)]);
  insertAt(object, index, nonSplicing) => jsvalue.callMethod(r"insertAt", [_(object), _(index), _(nonSplicing)]);
  item(index) => jsvalue.callMethod(r"item", [_(index)]);
  loadFromDatalessJSON(json, callback) => jsvalue.callMethod(r"loadFromDatalessJSON", [_(json), _(callback)]);
  loadFromJSON(json, callback) => jsvalue.callMethod(r"loadFromJSON", [_(json), _(callback)]);
  getObjects() => jsvalue.callMethod(r"getObjects", []);
  setOverlayImage(url, callback, options) => jsvalue.callMethod(r"setOverlayImage", [_(url), _(callback), _(options)]);
  observe(args) => jsvalue.callMethod(r"observe", args.map(_).toList());
  off(eventName, handler) => jsvalue.callMethod(r"off", [_(eventName), _(handler)]);
  on(args) => jsvalue.callMethod(r"on", args.map(_).toList());
  onBeforeScaleRotate() => jsvalue.callMethod(r"onBeforeScaleRotate", []);
  remove(object) => jsvalue.callMethod(r"remove", [_(object)]);
  renderAll(allOnTop) => jsvalue.callMethod(r"renderAll", [_(allOnTop)]);
  renderTop() => jsvalue.callMethod(r"renderTop", []);
  sendBackwards(object) => jsvalue.callMethod(r"sendBackwards", [_(object)]);
  sendToBack(object) => jsvalue.callMethod(r"sendToBack", [_(object)]);
  stopObserving(eventName, handler) => jsvalue.callMethod(r"stopObserving", [_(eventName), _(handler)]);
  straightenObject(object) => jsvalue.callMethod(r"straightenObject", [_(object)]);
  toDataURL(format, quality) => jsvalue.callMethod(r"toDataURL", [_(format), _(quality)]);
  toDataURLWithMultiplier(format, multiplier, quality) => jsvalue.callMethod(r"toDataURLWithMultiplier", [_(format), _(multiplier), _(quality)]);
  toDatalessJSON(propertiesToInclude) => jsvalue.callMethod(r"toDatalessJSON", [_(propertiesToInclude)]);
  toDatalessObject(propertiesToInclude) => jsvalue.callMethod(r"toDatalessObject", [_(propertiesToInclude)]);
  toJSON(propertiesToInclude) => jsvalue.callMethod(r"toJSON", [_(propertiesToInclude)]);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  toString() => jsvalue.callMethod(r"toString", []);
  getWidth() => jsvalue.callMethod(r"getWidth", []);
  setWidth(value) => jsvalue.callMethod(r"setWidth", [_(value)]);
  static supports(methodName) => _constructor.callMethod(r"supports", [_(methodName)]);
  static toGrayscale(canvasEl) => _constructor.callMethod(r"toGrayscale", [_(canvasEl)]);
}

class _Gradient extends _Proxy implements Gradient {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Gradient"];
  _Gradient(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toLiveGradient(ctx) => jsvalue.callMethod(r"toLiveGradient", [_(ctx)]);
  toObject() => jsvalue.callMethod(r"toObject", []);
  static forObject(obj, options) => _constructor.callMethod(r"forObject", [_(obj), _(options)]);
  static fromElement(el, instance) => _constructor.callMethod(r"fromElement", [_(el), _(instance)]);
}

class _Point extends _Proxy implements Point {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Point"];
  _Point(args) : jsvalue = new JsObject(_constructor, args);

  add(that) => jsvalue.callMethod(r"add", [_(that)]);
  addEquals(that) => jsvalue.callMethod(r"addEquals", [_(that)]);
  distanceFrom(that) => jsvalue.callMethod(r"distanceFrom", [_(that)]);
  divide(scalar) => jsvalue.callMethod(r"divide", [_(scalar)]);
  divideEquals(scalar) => jsvalue.callMethod(r"divideEquals", [_(scalar)]);
  eq(that) => jsvalue.callMethod(r"eq", [_(that)]);
  setFromPoint(that) => jsvalue.callMethod(r"setFromPoint", [_(that)]);
  gt(that) => jsvalue.callMethod(r"gt", [_(that)]);
  gte(that) => jsvalue.callMethod(r"gte", [_(that)]);
  init(x, y) => jsvalue.callMethod(r"init", [_(x), _(y)]);
  lerp(that, t) => jsvalue.callMethod(r"lerp", [_(that), _(t)]);
  lt(that) => jsvalue.callMethod(r"lt", [_(that)]);
  lte(that) => jsvalue.callMethod(r"lte", [_(that)]);
  max(that) => jsvalue.callMethod(r"max", [_(that)]);
  midPointFrom(that) => jsvalue.callMethod(r"midPointFrom", [_(that)]);
  min(that) => jsvalue.callMethod(r"min", [_(that)]);
  multiply(scalar) => jsvalue.callMethod(r"multiply", [_(scalar)]);
  multiplyEquals(scalar) => jsvalue.callMethod(r"multiplyEquals", [_(scalar)]);
  scalarAdd(scalar) => jsvalue.callMethod(r"scalarAdd", [_(scalar)]);
  scalarAddEquals(scalar) => jsvalue.callMethod(r"scalarAddEquals", [_(scalar)]);
  scalarSubtract(scalar) => jsvalue.callMethod(r"scalarSubtract", [_(scalar)]);
  scalarSubtractEquals(scalar) => jsvalue.callMethod(r"scalarSubtractEquals", [_(scalar)]);
  subtract(that) => jsvalue.callMethod(r"subtract", [_(that)]);
  subtractEquals(that) => jsvalue.callMethod(r"subtractEquals", [_(that)]);
  swap(that) => jsvalue.callMethod(r"swap", [_(that)]);
  toString() => jsvalue.callMethod(r"toString", []);
  setXY(x, y) => jsvalue.callMethod(r"setXY", [_(x), _(y)]);
}

class _Intersection extends _Proxy implements Intersection {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Intersection"];
  _Intersection(args) : jsvalue = new JsObject(_constructor, args);

  appendPoint(point) => jsvalue.callMethod(r"appendPoint", [_(point)]);
  appendPoints(points) => jsvalue.callMethod(r"appendPoints", [_(points)]);
  init(status) => jsvalue.callMethod(r"init", [_(status)]);
  static intersectLineLine(a1, a2, b1, b2) => _constructor.callMethod(r"intersectLineLine", [_(a1), _(a2), _(b1), _(b2)]);
  static intersectLinePolygon(a1, a2, points) => _constructor.callMethod(r"intersectLinePolygon", [_(a1), _(a2), _(points)]);
  static intersectPolygonPolygon(points1, points2) => _constructor.callMethod(r"intersectPolygonPolygon", [_(points1), _(points2)]);
  static intersectPolygonRectangle(points, r1, r2) => _constructor.callMethod(r"intersectPolygonRectangle", [_(points), _(r1), _(r2)]);
}

class _Color extends _Proxy implements Color {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Color"];
  _Color(color) : jsvalue = new JsObject(_constructor, [_(color)]);

  getAlpha() => jsvalue.callMethod(r"getAlpha", []);
  setAlpha(alpha) => jsvalue.callMethod(r"setAlpha", [_(alpha)]);
  overlayWith(otherColor) => jsvalue.callMethod(r"overlayWith", [_(otherColor)]);
  getSource() => jsvalue.callMethod(r"getSource", []);
  setSource(source) => jsvalue.callMethod(r"setSource", [_(source)]);
  toBlackWhite(threshold) => jsvalue.callMethod(r"toBlackWhite", [_(threshold)]);
  toGrayscale() => jsvalue.callMethod(r"toGrayscale", []);
  toHex() => jsvalue.callMethod(r"toHex", []);
  toRgb() => jsvalue.callMethod(r"toRgb", []);
  toRgba() => jsvalue.callMethod(r"toRgba", []);
  static fromHex(color) => _constructor.callMethod(r"fromHex", [_(color)]);
  static fromRgb(color) => _constructor.callMethod(r"fromRgb", [_(color)]);
  static fromRgba(color) => _constructor.callMethod(r"fromRgba", [_(color)]);
  static fromSource(source) => _constructor.callMethod(r"fromSource", [_(source)]);
  static sourceFromHex(color) => _constructor.callMethod(r"sourceFromHex", [_(color)]);
  static sourceFromRgb(color) => _constructor.callMethod(r"sourceFromRgb", [_(color)]);
}

class _FreeDrawing extends _Proxy implements FreeDrawing {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["FreeDrawing"];
  _FreeDrawing(fabricCanvas) : jsvalue = new JsObject(_constructor, [_(fabricCanvas)]);

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  convertPointsToSVGPath(points, minX, maxX, minY, maxY) => jsvalue.callMethod(r"convertPointsToSVGPath", [_(points), _(minX), _(maxX), _(minY), _(maxY)]);
  getPathBoundingBox(points) => jsvalue.callMethod(r"getPathBoundingBox", [_(points)]);
}

class _Canvas extends _StaticCanvas implements Canvas {
  static JsObject _constructor = context["fabric"]["Canvas"];
  _Canvas(el, options) : super.withValue(new JsObject(_constructor, [_(el), _(options)]));

  getActiveGroup() => jsvalue.callMethod(r"getActiveGroup", []);
  setActiveGroup(group) => jsvalue.callMethod(r"setActiveGroup", [_(group)]);
  getActiveObject() => jsvalue.callMethod(r"getActiveObject", []);
  setActiveObject(object, e) => jsvalue.callMethod(r"setActiveObject", [_(object), _(e)]);
  containsPoint(e, target) => jsvalue.callMethod(r"containsPoint", [_(e), _(target)]);
  deactivateAll() => jsvalue.callMethod(r"deactivateAll", []);
  deactivateAllWithDispatch() => jsvalue.callMethod(r"deactivateAllWithDispatch", []);
  discardActiveGroup() => jsvalue.callMethod(r"discardActiveGroup", []);
  discardActiveObject() => jsvalue.callMethod(r"discardActiveObject", []);
  drawDashedLine(ctx, x, y, x2, y2, da) => jsvalue.callMethod(r"drawDashedLine", [_(ctx), _(x), _(y), _(x2), _(y2), _(da)]);
  findTarget(e, skipGroup) => jsvalue.callMethod(r"findTarget", [_(e), _(skipGroup)]);
  getPointer(e) => jsvalue.callMethod(r"getPointer", [_(e)]);
  getSelectionContext() => jsvalue.callMethod(r"getSelectionContext", []);
  getSelectionElement() => jsvalue.callMethod(r"getSelectionElement", []);
  toString() => jsvalue.callMethod(r"toString", []);
  static bind(args) => _constructor.callMethod(r"bind", args.map(_).toList());
  static supports(methodName) => _constructor.callMethod(r"supports", [_(methodName)]);
  static toGrayscale(canvasEl) => _constructor.callMethod(r"toGrayscale", [_(canvasEl)]);
}

class _Line extends _Object implements Line {
  static JsObject _constructor = context["fabric"]["Line"];
  _Line(points, options) : super.withValue(new JsObject(_constructor, [_(points), _(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Circle extends _Object implements Circle {
  static JsObject _constructor = context["fabric"]["Circle"];
  _Circle(options) : super.withValue(new JsObject(_constructor, [_(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  getRadiusX() => jsvalue.callMethod(r"getRadiusX", []);
  getRadiusY() => jsvalue.callMethod(r"getRadiusY", []);
  setRadius(value) => jsvalue.callMethod(r"setRadius", [_(value)]);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Triangle extends _Object implements Triangle {
  static JsObject _constructor = context["fabric"]["Triangle"];
  _Triangle(options) : super.withValue(new JsObject(_constructor, [_(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Ellipse extends _Object implements Ellipse {
  static JsObject _constructor = context["fabric"]["Ellipse"];
  _Ellipse(options) : super.withValue(new JsObject(_constructor, [_(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  render(ctx, noTransform) => jsvalue.callMethod(r"render", [_(ctx), _(noTransform)]);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Rect extends _Object implements Rect {
  static JsObject _constructor = context["fabric"]["Rect"];
  _Rect(options) : super.withValue(new JsObject(_constructor, [_(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Polyline extends _Object implements Polyline {
  static JsObject _constructor = context["fabric"]["Polyline"];
  _Polyline(points, options) : super.withValue(new JsObject(_constructor, [_(points), _(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Polygon extends _Object implements Polygon {
  static JsObject _constructor = context["fabric"]["Polygon"];
  _Polygon(points, options) : super.withValue(new JsObject(_constructor, [_(points), _(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _PathGroup extends _Path implements PathGroup {
  static JsObject _constructor = context["fabric"]["PathGroup"];
  _PathGroup(paths, options) : super.withValue(new JsObject(_constructor, [_(paths), _(options)]));

  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  getObjects() => jsvalue.callMethod(r"getObjects", []);
  render(ctx) => jsvalue.callMethod(r"render", [_(ctx)]);
  isSameColor() => jsvalue.callMethod(r"isSameColor", []);
  toDatalessObject(propertiesToInclude) => jsvalue.callMethod(r"toDatalessObject", [_(propertiesToInclude)]);
  toGrayscale() => jsvalue.callMethod(r"toGrayscale", []);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  toString() => jsvalue.callMethod(r"toString", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Group extends _Object implements Group {
  static JsObject _constructor = context["fabric"]["Group"];
  _Group(objects, options) : super.withValue(new JsObject(_constructor, [_(objects), _(options)]));

  activateAllObjects() => jsvalue.callMethod(r"activateAllObjects", []);
  add(object) => jsvalue.callMethod(r"add", [_(object)]);
  addWithUpdate(object) => jsvalue.callMethod(r"addWithUpdate", [_(object)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  complexity() => jsvalue.callMethod(r"complexity", []);
  contains(object) => jsvalue.callMethod(r"contains", [_(object)]);
  containsPoint(point) => jsvalue.callMethod(r"containsPoint", [_(point)]);
  destroy() => jsvalue.callMethod(r"destroy", []);
  forEachObject(callback, context) => jsvalue.callMethod(r"forEachObject", [_(callback), _(context)]);
  get(prop) => jsvalue.callMethod(r"get", [_(prop)]);
  hasMoved() => jsvalue.callMethod(r"hasMoved", []);
  item(index) => jsvalue.callMethod(r"item", [_(index)]);
  setObjectsCoords() => jsvalue.callMethod(r"setObjectsCoords", []);
  getObjects() => jsvalue.callMethod(r"getObjects", []);
  remove(object) => jsvalue.callMethod(r"remove", [_(object)]);
  removeWithUpdate(object) => jsvalue.callMethod(r"removeWithUpdate", [_(object)]);
  render(ctx, noTransform) => jsvalue.callMethod(r"render", [_(ctx), _(noTransform)]);
  saveCoords() => jsvalue.callMethod(r"saveCoords", []);
  size() => jsvalue.callMethod(r"size", []);
  toGrayscale() => jsvalue.callMethod(r"toGrayscale", []);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  toString() => jsvalue.callMethod(r"toString", []);
  static fromObject(object, callback) => _constructor.callMethod(r"fromObject", [_(object), _(callback)]);
}

class _Image extends _Object implements Image {
  static JsObject _constructor = context["fabric"]["Image"];
  _Image(element, options) : super.withValue(new JsObject(_constructor, [_(element), _(options)]));

  applyFilters(callback) => jsvalue.callMethod(r"applyFilters", [_(callback)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  clone(callback, propertiesToInclude) => jsvalue.callMethod(r"clone", [_(callback), _(propertiesToInclude)]);
  complexity() => jsvalue.callMethod(r"complexity", []);
  getElement() => jsvalue.callMethod(r"getElement", []);
  setElement(element) => jsvalue.callMethod(r"setElement", [_(element)]);
  getOriginalSize() => jsvalue.callMethod(r"getOriginalSize", []);
  render(ctx, noTransform) => jsvalue.callMethod(r"render", [_(ctx), _(noTransform)]);
  getSrc() => jsvalue.callMethod(r"getSrc", []);
  getSvgSrc() => jsvalue.callMethod(r"getSvgSrc", []);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  toString() => jsvalue.callMethod(r"toString", []);
  static fromElement(element, callback, options) => _constructor.callMethod(r"fromElement", [_(element), _(callback), _(options)]);
  static fromObject(object, callback) => _constructor.callMethod(r"fromObject", [_(object), _(callback)]);
  static fromURL(url, callback, imgOptions) => _constructor.callMethod(r"fromURL", [_(url), _(callback), _(imgOptions)]);
}

class _Text extends _Object implements Text {
  static JsObject _constructor = context["fabric"]["Text"];
  _Text(text, options) : super.withValue(new JsObject(_constructor, [_(text), _(options)]));

  setColor(value) => jsvalue.callMethod(r"setColor", [_(value)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  setFontsize(value) => jsvalue.callMethod(r"setFontsize", [_(value)]);
  render(ctx, noTransform) => jsvalue.callMethod(r"render", [_(ctx), _(noTransform)]);
  getText() => jsvalue.callMethod(r"getText", []);
  setText(value) => jsvalue.callMethod(r"setText", [_(value)]);
  toObject(propertiesToInclude) => jsvalue.callMethod(r"toObject", [_(propertiesToInclude)]);
  toSVG() => jsvalue.callMethod(r"toSVG", []);
  toString() => jsvalue.callMethod(r"toString", []);
  static fromElement(element, options) => _constructor.callMethod(r"fromElement", [_(element), _(options)]);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Grayscale extends _Proxy implements Grayscale {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Grayscale"];
  _Grayscale() : jsvalue = new JsObject(_constructor, []);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject() => _constructor.callMethod(r"fromObject", []);
}

class _RemoveWhite extends _Proxy implements RemoveWhite {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["RemoveWhite"];
  _RemoveWhite(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Invert extends _Proxy implements Invert {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Invert"];
  _Invert() : jsvalue = new JsObject(_constructor, []);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject() => _constructor.callMethod(r"fromObject", []);
}

class _Sepia extends _Proxy implements Sepia {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Sepia"];
  _Sepia() : jsvalue = new JsObject(_constructor, []);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject() => _constructor.callMethod(r"fromObject", []);
}

class _Sepia2 extends _Proxy implements Sepia2 {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Sepia2"];
  _Sepia2() : jsvalue = new JsObject(_constructor, []);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject() => _constructor.callMethod(r"fromObject", []);
}

class _Brightness extends _Proxy implements Brightness {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Brightness"];
  _Brightness(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Noise extends _Proxy implements Noise {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Noise"];
  _Noise(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _GradientTransparency extends _Proxy implements GradientTransparency {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["GradientTransparency"];
  _GradientTransparency(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Tint extends _Proxy implements Tint {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Tint"];
  _Tint(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Convolute extends _Proxy implements Convolute {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Convolute"];
  _Convolute(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _Pixelate extends _Proxy implements Pixelate {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["Image"]["filters"]["Pixelate"];
  _Pixelate(options) : jsvalue = new JsObject(_constructor, [_(options)]);

  applyTo(canvasEl) => jsvalue.callMethod(r"applyTo", [_(canvasEl)]);
  callSuper(args) => jsvalue.callMethod(r"callSuper", args.map(_).toList());
  toJSON() => jsvalue.callMethod(r"toJSON", []);
  static fromObject(object) => _constructor.callMethod(r"fromObject", [_(object)]);
}

class _request extends _Proxy implements request {
  var jsvalue;
  static JsObject _constructor = context["fabric"]["util"]["request"];
  _request(url, options) : jsvalue = new JsObject(_constructor, [_(url), _(options)]);

}

class _Size extends _Proxy implements Size {
  var jsvalue;
  static JsObject _constructor = context["Cufon"]["CSS"]["Size"];
  _Size(value, base) : jsvalue = new JsObject(_constructor, [_(value), _(base)]);

  get value => jsvalue["value"];
  set value(x) => jsvalue["value"] = x;
  get unit => jsvalue["unit"];
  set unit(x) => jsvalue["unit"] = x;
  get convert => jsvalue["convert"];
  set convert(x) => jsvalue["convert"] = x;
  get convertFrom => jsvalue["convertFrom"];
  set convertFrom(x) => jsvalue["convertFrom"] = x;
}

