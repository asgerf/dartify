import 'fabricjs.dart' as fabric;

import 'dart:js';

main() {
  var canvas = new fabric.Canvas('c', null);

//  fabric.Object.prototype.transparentCorners = false; // TODO support this?

  var rect1 = new fabric.Rect({ // TODO: infer named parameters from source code?
    'width': 200,
    'height': 100,
    'left': 0,
    'top': 50,
    'angle': 30,
    'fill': 'rgba(255,0,0,0.5)'
  });

  var rect2 = new fabric.Rect({
    'width': 100,
    'height': 100,
    'left': 350,
    'top': 250,
    'angle': -10,
    'fill': 'rgba(0,200,0,0.5)'
  });

  var rect3 = new fabric.Rect({
    'width': 50,
    'height': 100,
    'left': 275,
    'top': 350,
    'angle': 45,
    'fill': 'rgba(0,0,200,0.5)'
  });

  var circle = new fabric.Circle({
    'radius': 50,
    'left': 275,
    'top': 75,
    'fill': '#aac'
  });

  var triangle = new fabric.Triangle({
    'width': 100,
    'height': 100,
    'left': 50,
    'top': 300,
    'fill': '#cca'
  });

  canvas.add([rect1, rect2, rect3, circle, triangle]);

  void onChange(options) {
    options['target'].callMethod('setCoords', []);
    canvas.forEachObject((obj, a, b) {
      if (obj == options['target']) return;
      obj.callMethod('setOpacity', [options['target'].callMethod('intersectsWithObject', [obj]) ? 0.5 : 1]);
    }, null);
  }

  canvas.on(['object:moving', onChange]);
}
