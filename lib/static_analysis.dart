import 'jsnap.dart';

abstract class AbstractHeap {
  Snapshot snapshot;

  AbstractHeap(this.snapshot);

  AbstractValue createUnknown();
  AbstractValue createNothing();

  bool aliased(AbstractValue x, AbstractValue y);
}

abstract class AbstractValue { }

class Singleton extends AbstractValue {
  Obj object;

  Singleton(this.object);
}

class TypeVar extends AbstractValue {
  TypeVar _parent;
  int _rank = 0;

  TypeVar get root {
    if (_parent == null) {
      return this;
    } else {
      return _parent = _parent.root;
    }
  }

  Map<String, AbstractValue> _properties = <String, AbstractValue>{};
  Set<dynamic> _prototypes = new Set<dynamic>();
  Set<dynamic> _primitives = new Set<dynamic>();

  void addPrimitive(x) {
    assert(x is num || x is String || x is bool || x == null || x is Undefined || x is Obj);
    _primitives.add(x);
  }
}

class Unifier {
  List<TypeVar> worklist = <TypeVar>[];

  void iterate() {
    while (worklist.isNotEmpty) {
      TypeVar x = worklist.removeLast();
      TypeVar y = worklist.removeLast();
      unify(x, y);
    }
  }

  void unifyLater(TypeVar v1, TypeVar v2) {
    if (v1 == v2) return; // Optimization, same check would happen later.
    worklist.add(v1);
    worklist.add(v2);
  }

  void unify(TypeVar v1, TypeVar v2) {
    v1 = v1.root;
    v2 = v2.root;
    if (v1 == v2) return;
    if (v1.rank < v2.rank) {
      var tmp = v1;
      v1 = v2;
      v2 = tmp;
    } else if (v1.rank == v2.rank) {
      v1.rank++;
    }
    v2.parent = v1;
    v1._properties = unifyMaps(v1._properties, v2._properties);
    v1._prototypes = mergeSets(v1._prototypes, v2._prototypes);
    v1._primitives = mergeSets(v1._primitives, v2._primitives);
    v2._properties = null;
    v2._prototypes = null;
    v2._primitives = null;
  }

  Map unifyMaps(Map map1, Map map2) {
    if (map1.length < map2.length) {
      var tmp = map1;
      map1 = map2;
      map2 = tmp;
    }
    for (var key in map2.keys) {
      var value = map2[key];
      var existingValue = map1[key];
      if (existingValue == null) {
        map1[key] = value;
      } else if (value == existingValue) {
        // Nothing to do.
      } else if (value is TypeVar && existingValue is TypeVar) {
        unifyLater(existingValue, value);
      } else if (value is TypeVar) {
        value = value.root;
        map1[key] = value;
        value.addPrimitive(existingValue);
      }
    }
    return map1;
  }

  Set mergeSets(Set set1, Set set2) {
    if (set1.length < set2.length) {
      set2.addAll(set1);
      return set2;
    } else {
      set1.addAll(set2);
      return set1;
    }
  }
}
