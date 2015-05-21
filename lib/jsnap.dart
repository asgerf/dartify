library jsnap;

import 'dart:convert' show JSON;

/// An object from a jsnap snapshot.
class Obj {
  /// Index of this object in the heap.
  int id;
  
  /// What type of function this is, or `null` if not a function.
  Func function;
  
  /// Pointer to the enclosing environment object, or `null` for
  /// objects without an environment pointer.
  /// 
  /// User function objects and environment objects may have an environment pointer.
  Obj env;
  
  /// Pointer to the prototype object. Is always `null` for environment objects.
  Obj prototype;
  
  /// List of properties on this object, as reported by `Object.getOwnPropertyNames`.
  List<Property> properties = <Property>[];
  
  /// Returns the property with the given name or `null` if not found. The prototype chain is not used.
  ///
  /// Uses linear search so do not expect this to be fast.
  Property operator[](String name) {
    for (Property prop in properties) {
      if (prop.name == name) return prop;
    }
    return null;
  }
  
  String toString() => 'obj($id,$function)';
  
  Obj(this.id);
}

class Property {
  String name;
  bool writeable;
  bool configurable;
  bool enumerable;
  dynamic value;
  Obj get;
  Obj set;
  
  Property(this.name, {this.value, this.get, this.set, this.writeable:true, this.configurable:true, this.enumerable:true});
  
  String toString() {
    if (get != null || set != null) {
      return '$name: get $get / set $set';
    } else {
      return '$name: $value';
    }
  }
  
  bool get isAccessor => get != null || set != null;
}

/// The `undefined` value.
class Undefined {
  const Undefined();

  String toString() => 'undefined';
}

abstract class Func {}

class UserFunc extends Func {
  int id;
  
  UserFunc(this.id);
  
  String toString() => 'function($id)';
}

class NativeFunc extends Func {
  String id;
  
  NativeFunc(this.id);
  
  String toString() => id;
}

class BindFunc extends Func {
  dynamic target;
  List<dynamic> arguments;
  
  BindFunc(this.target, this.arguments);
  
  String toString() => 'bind($target, ${arguments.join(',')})';
}

class UnknownFunc extends Func {
}

class NoValue {
  const NoValue();
}

class Snapshot {
  Obj global;
  List<Obj> heap;
  
  dynamic operator[](String path) {
    dynamic currentValue = global;
    for (String part in path.split('.')) {
      if (currentValue is! Obj) return const NoValue();
      Obj object = currentValue;
      Property prty = object[part];
      if (prty == null) return const NoValue();
      if (prty.isAccessor) return const NoValue();
      currentValue = prty.value;
    }
    return currentValue;
  }
  
  Snapshot.fromJSON(Map snapshotJson) {
    List<Obj> json2obj; // Map from JSNAP heap index to object. Because we remove holes, these do not match [heap].
    heap = <Obj>[];
    
    dynamic parseValue(dynamic value) {
      if (value is Map) {
        int key = value['key'];
        if (key != null) return json2obj[key];
        if (value.containsKey('isUndefined')) return const Undefined();
      } else if (value is num || value is bool || value is String || value == null) {
        return value;
      }
      throw 'Unrecognized value: $value';
    }
    
    List<dynamic> parseValues(List json) => json.map(parseValue).toList();
    
    Func parseFunction(Map json) {
      if (json == null) return null;
      switch (json['type']) {
        case 'user': return new UserFunc(json['id']);
        case 'native': return new NativeFunc(json['id']);
        case 'bind': return new BindFunc(parseValue(json['target']), parseValues(json['arguments']));
        case 'unknown': return new UnknownFunc();
        default: throw 'Unrecognized function type: $json';
      }
    }
    
    Property parseProperty(Map json) {
      return new Property(json['name'], 
          value: parseValue(json['value']),
          get: parseValue(json['get']),
          set: parseValue(json['set']),
          writeable: parseValue(json['writeable']),
          configurable: parseValue(json['configurable']),
          enumerable: parseValue(json['enumerable']));
    }
    
    List<Property> parseProperties(List json) => json.map(parseProperty).toList();

    List jsonHeap = snapshotJson['heap'];
    json2obj = new List(jsonHeap.length);
    for (int i=0; i<jsonHeap.length; i++) {
      if (jsonHeap[i] == null) continue;
      Obj object = new Obj(heap.length);
      heap.add(object);
      json2obj[i] = object;
    }
    int index = 0;
    for (int i=0; i<jsonHeap.length; i++) {
      if (jsonHeap[i] == null) continue;
      Map json = jsonHeap[i];
      Obj obj = heap[index++];
      obj.function = parseFunction(json['function']);
      obj.env = parseValue(json['env']);
      obj.prototype = parseValue(json['prototype']);
      obj.properties = parseProperties(json['properties']);
    }
    
    global = json2obj[snapshotJson['global']];
  }
}

Snapshot parseSnapshot(String text) {
  return new Snapshot.fromJSON(JSON.decode(text));
}
