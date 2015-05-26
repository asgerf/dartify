library dartify;

import 'bimap.dart';
import 'model.dart' hide Method;
import 'model.dart' as model;
import 'package:parsejs/parsejs.dart' hide Program, Property;
import 'package:parsejs/parsejs.dart' as ast;
import 'jsnap.dart';

Program dartify(Snapshot snapshot, ast.Program ast) {
  Namer namer = new Namer(snapshot);
  AstInfo astInfo = new AstInfo(ast);
  SnapshotInfo snapshotInfo = new SnapshotInfo(snapshot);

  // Objects with a UserFunc function.
  List<Obj> userFunctions = snapshot.heap.where((obj) => obj.function is UserFunc).toList();

  // Identify class constructors.
  Set<Obj> constructors = new Set<Obj>();

  // Functions that are pointed at by an inherited 'constructor' property are constructors.
  for (Obj object in snapshot.heap) {
    Obj proto = object.prototype;
    if (proto == null) continue;
    Property constructor = proto['constructor'];
    if (constructor == null) continue;
    if (constructor.value is! Obj) continue;
    Obj constructorValue = constructor.value;
    if (constructorValue.function is! UserFunc) continue;
    constructors.add(constructor.value);
  }

  // Functions whose prototype companion objects have user functions on them are constructors.
  for (Obj functionObj in userFunctions) {
    Property protoPrty = functionObj['prototype'];
    if (protoPrty == null) continue;
    if (protoPrty.value is! Obj) continue;
    Obj proto = protoPrty.value;
    for (Property prty in proto.properties) {
      if (prty.name == 'constructor') continue;
      if (prty.value is Obj) {
        Obj value = prty.value;
        if (value.function is UserFunc) {
          constructors.add(functionObj);
          break;
        }
      }
    }
  }

  // Some classes might not have had instances when the snapshot was created.
  // Look in the AST for `new E()` expressions, and try to find the corresponding
  // JS constructor for E and mark it as a constructor.
  for (Obj functionObj in userFunctions) {
    UserFunc function = functionObj.function;
    ast.FunctionNode node = astInfo.getFunction(functionObj);
    new ConstructorFinder(snapshot, constructors).analyzeFunction(node, functionObj.env, null, []); // XXX: set thisArg based on parents in heap
  }

  // Get rid of constructors that aren't accessible.
  constructors.removeWhere((x) => !snapshotInfo.isAccessible(x));

  Obj getSuperClassConstructor(Obj constructor) {
    Property prototypePrty = constructor['prototype'];
    if (prototypePrty == null || prototypePrty.value is! Obj) return null;
    Obj prototype = prototypePrty.value;
    Obj superPrototype = prototype.prototype;
    if (superPrototype == null) return null;
    Property superConstructorPrty = superPrototype['constructor'];
    if (superConstructorPrty == null) return null;
    if (!constructors.contains(superConstructorPrty.value)) return null;
    return superConstructorPrty.value;
  }

  RedirectClosure redirects = new RedirectClosure(snapshot, snapshotInfo, ast, astInfo);

  Parameters getParameters(Obj function, dynamic thisArgument) {
    Method target = redirects.getTransitiveRedirect(new Method(function, thisArgument));
    FunctionNode node = astInfo.getFunction(target.function);
    List<String> params = node.params.map((Name name) => name.value);
    return new Parameters(params, isVarArgs: redirects.usesArgumentsArray.contains(target));
  }

  Program program = new Program();
  program.libraryName = getLibraryNameFromFilename(ast.filename);

  Bimap<Obj, Class> obj2class = new Bimap<Obj, Class>();
  for (Obj constructor in constructors) {
    obj2class[constructor] = new Class();
  }

  // Generate each class.
  for (Obj constructor in constructors) {
    Class clazz = obj2class[constructor];
    clazz.name = namer.getName(constructor);
    clazz.jsConstructor = namer.getPathAsList(constructor);
    clazz.superClass = obj2class[getSuperClassConstructor(constructor)];
    Property prototypePrty = constructor['prototype'];
    if (prototypePrty == null) continue;
    if (prototypePrty.value is! Obj) continue;
    Obj prototype = prototypePrty.value;
    Method constructorAsMethod = new Method(constructor, prototype);

    // Generate the constructor.
    clazz.constructorParams = getParameters(constructor, prototype);

    // Generate members.
    for (Property prty in prototype.properties) {
      String member = prty.name;
      if (member == 'constructor') continue;
      if (member.startsWith('_')) continue;
      if (prty.isAccessor) continue; // TODO
      if (member.startsWith('init') && redirects.isRedirectTo(constructorAsMethod, prty.value)) continue; // Ignore constructor stubs.
      var value = prty.value;
      if (value is Obj && value.function is UserFunc) {
        Parameters params = getParameters(value, prototype);
        clazz.instanceMethods.add(new model.Method(member, params));
      }
    }

    // Print static members
    for (Property prty in constructor.properties) {
      String member = prty.name;
      if (prty.value is! Obj) continue;
      Obj value = prty.value;
      if (value.function is! UserFunc) continue;
      if (constructors.contains(value)) continue; // Superclass pointer.
      Parameters params = getParameters(value, constructor);
      clazz.staticMethods.add(new model.Method(member, params));
    }

    program.classes.add(clazz);
  }

  return program;
}

/// Some generic information about the AST.
class AstInfo extends RecursiveVisitor {
  Bimap<int, FunctionNode> id2function = new Bimap<int, FunctionNode>();
  int nextFunctionId = 1; // The first function must have ID 1.

  AstInfo(ast.Program program) {
    visit(program);
  }

  @override
  visitFunctionNode(FunctionNode node) {
    id2function[nextFunctionId++] = node;
    node.forEach(visit);
  }

  FunctionNode getFunction(Obj object) => id2function[object.function.id];
}

/// Some generic information about the heap snapshot.
class SnapshotInfo {
  Snapshot snapshot;
  List<bool> accessible;

  /// True if [object] is accessible using ordinary property accesses. That is, no prototype links,
  /// environment pointers, or getter/setter accessors are needed to get to the object.
  bool isAccessible(Obj object) {
    return accessible[object.id];
  }

  SnapshotInfo(this.snapshot) {
    accessible = new List.filled(snapshot.heap.length, false);
    void search(x) {
      if (x is! Obj) return;
      Obj object = x;
      if (accessible[object.id]) return;
      accessible[object.id] = true;
      for (Property prty in object.properties) {
        search(prty.value);
      }
    }
    search(snapshot.global);
  }
}

/// Traverses the heap and names objects based on the shortest path it finds to that object starting
/// at the global object.
///
/// Two breath-first searches are employed. The first pass only traverses edges with a "nice" name
/// to avoid getting ugly names like environment pointers and prototypes. The second pass allows any
/// name, so we can name objects that can't be reached any other way.
class Namer {
  Snapshot snapshot;
  List<Obj> parents;
  List<String> names;
  List<Obj> queue = <Obj>[];

  Namer(this.snapshot) {
    parents = new List(snapshot.heap.length);
    names = new List(snapshot.heap.length);
    names[snapshot.global.id] = '<Global>';
    queue.add(snapshot.global);

    for (int i=0; i<queue.length; i++) {
      search(queue[i], onlyNiceNames: true);
    }
    for (int i=0; i<queue.length; i++) {
      search(queue[i], onlyNiceNames: false);
    }

    // If for some reason an object could not be found, just name it <Unknown>.
    for (Obj object in snapshot.heap) {
      if (names[object.id] == null) {
        names[object.id] = '<Unknown>';
      }
    }
    queue.clear();
  }

  /// Given an [object] that already has a name, enqueue objects along its outgoing edges.
  void search(Obj object, {bool onlyNiceNames}) {
    void tryValue(x, String name) {
      if (x is Obj && names[x.id] == null) {
        names[x.id] = name;
        parents[x.id] = object;
        queue.add(x);
      }
    }
    for (Property binding in object.properties) {
      if (onlyNiceNames && !isNiceName(binding.name)) continue;
      tryValue(binding.value, binding.name);
      tryValue(binding.get, 'get#' + binding.name);
      tryValue(binding.set, 'set#' + binding.name);
    }
    if (!onlyNiceNames) {
      tryValue(object.prototype, '[prototype]');
      tryValue(object.env, '[env]');
    }
  }

  bool isNiceName(String name) => !name.startsWith('_') && !name.startsWith('[');

  static final RegExp identifierRex = new RegExp(r'[a-zA-Z_][a-zA-Z0-9_$]*$|\[prototype\]|\[env\]');
  bool isIdentifier(String name) => identifierRex.matchAsPrefix(name) != null;

  /// The short name for [object], which is the last part of its path.
  String getName(Obj object) => names[object.id];

  /// A short and nice path from the global object that leads to [object].
  String getPath(Obj object) {
    String name = getName(object);
    if (parents[object.id] == snapshot.global || parents[object.id] == null) {
      return name;
    } else if (!isIdentifier(name)) {
      return getPath(parents[object.id]) + "['$name']";
    } else {
      return getPath(parents[object.id]) + '.' + name;
    }
  }

  List<String> getPathAsList(Obj object) {
    Obj parent = parents[object.id];
    if (parent == null) return <String>[];
    List<String> paths = getPathAsList(parent);
    paths.add(names[object.id]);
    return paths;
  }

  String getCompactPath(Obj object) => getPath(object).replaceAll('.prototype.', '::');
}

class ArgumentsArray {
  List values;
  ArgumentsArray(this.values);
}

/// Runs a single pass over the AST of a function while computing the values of expressions and values.
///
/// The values obtained are not sound; merely crude approximations that should be enough to resolve simple
/// direct access to objects in the snapshot.
class OnePassAbstractEvaluator extends RecursiveVisitor {
  Snapshot snapshot;
  Obj outerEnvironment;
  List<Map<String, dynamic>> environment = [{}];
  dynamic thisValue;
  ArgumentsArray argumentsArray;

  OnePassAbstractEvaluator(this.snapshot);

  void analyzeFunction(FunctionNode node, Obj environment, dynamic thisValue, List arguments) {
    outerEnvironment = environment;
    argumentsArray = new ArgumentsArray(arguments);
    pushEnvironment(node.environment);
    updateEnvironment('arguments', argumentsArray);
    this.thisValue = thisValue;
    for (int i = 0; i < arguments.length; i++) {
      if (node.params.length < i) {
        updateEnvironment(node.params[i].value, arguments[i]);
      }
    }
    node.forEach(visit);
    popEnvironment();
  }

  void pushEnvironment(Iterable<String> names) {
    var env = {};
    for (String key in names) {
      env[key] = const NoValue();
    }
    environment.add(env);
  }

  void popEnvironment() {
    environment.removeLast();
  }

  void updateEnvironment(String variable, dynamic value) {
    for (int i = environment.length - 1; i >= 0; i--) {
      if (environment[i].containsKey(variable)) {
        environment[i][variable] = value;
        return;
      }
    }
    // If not found, use the outermost local environment to shadow the value in the heap.
    // Do not modify the heap structure.
    environment[0][variable] = value;
  }

  dynamic lookupEnvironment(String variable) {
    // Look in the local environment stack.
    for (int i = environment.length - 1; i >= 0; i--) {
      if (environment[i].containsKey(variable)) {
        return environment[i][variable];
      }
    }
    // Look in the outer environment chain.
    Obj env = outerEnvironment;
    while (env != null) {
      Property binding = env[variable];
      if (binding != null) {
        return binding.value;
      }
      env = env.env;
    }
    // Look in global object.
    return lookup(snapshot.global, variable);
  }

  dynamic lookup(Obj object, String name) {
    while (object != null) { // Loop over prototypes
      Property binding = object[name];
      if (binding != null) {
        if (binding.isAccessor) return const NoValue(); // This function will be analyzed separately.
        return binding.value;
      }
      object = object.prototype;
    }
    return const Undefined();
  }

  visitFunctionNode(FunctionNode node) {
    pushEnvironment(node.environment);
    updateEnvironment('arguments', new ArgumentsArray([]));
    node.forEach(visit);
    popEnvironment();
  }

  @override
  visitVariableDeclarator(VariableDeclarator node) {
    var value = node.init == null ? const Undefined() : visit(node.init);
    updateEnvironment(node.name.value, value);
  }

  @override
  visitAssignment(AssignmentExpression node) {
    if (node.operator != '=') {
      node.forEach(visit);
      return const NoValue();
    }
    if (node.left is NameExpression) {
      NameExpression left = node.left;
      var value = visit(node.right);
      updateEnvironment(left.name.value, value);
      return value;
    } else {
      visit(node.left);
      return visit(node.right);
    }
  }

  @override
  visitMember(MemberExpression node) {
    var object = visit(node.object);
    if (object is Obj) {
      return lookup(object, node.property.value);
    }
    return const NoValue();
  }

  @override
  visitIndex(IndexExpression node) {
    var object = visit(node.object);
    var index = visit(node.property);
    if (object is Obj && (index is num || index is String)) {
      return lookup(object, index.toString());
    }
    return const NoValue();
  }

  @override
  visitNameExpression(NameExpression node) {
    return lookupEnvironment(node.name.value);
  }

  @override
  visitLiteral(LiteralExpression node) {
    return node.value;
  }

  @override
  visitThis(ThisExpression node) {
    return thisValue;
  }

  @override
  defaultNode(Node node) {
    node.forEach(visit);
    return const NoValue();
  }
}

/// Analyzes the AST of a method in a given environment and marks objects that appear to be used as constructors.
class ConstructorFinder extends OnePassAbstractEvaluator {
  Set<Obj> constructors;

  ConstructorFinder(Snapshot snapshot, this.constructors) : super(snapshot);

  @override
  visitCall(CallExpression node) {
    var callee = visit(node.callee);
    node.arguments.forEach(visit);
    if (node.isNew) {
      markAsConstructor(callee);
    }
    return super.visitCall(node);
  }

  void markAsConstructor(value) {
    if (value is! Obj) return;
    if (value.function is! UserFunc) return;
    constructors.add(value);
  }
}

/// Function object and a bound `this` argument.
class Method {
  Obj function;
  Obj thisArg;

  Method(this.function, this.thisArg);

  bool operator==(other) => other is Method && function == other.function && thisArg == other.thisArg;

  int get hashCode => function.hashCode * 13 + thisArg.hashCode * 171;
}

class RedirectClosure {

  Snapshot snapshot;
  SnapshotInfo snapshotInfo;
  ast.Program ast;
  AstInfo astInfo;
  Map<Method, Method> redirectMap = <Method, Method>{};
  Set<Method> usesArgumentsArray = new Set<Method>();

  RedirectClosure(this.snapshot, this.snapshotInfo, this.ast, this.astInfo);

  Method getImmediateRedirect(Method method) {
    if (method.function.function is! UserFunc) return null;
    if (redirectMap.containsKey(method)) return redirectMap[method];
    FunctionNode node = astInfo.getFunction(method.function);
    RedirectFinder finder = new RedirectFinder(snapshot);
    finder.analyzeFunction(node, method.function.env, method.thisArg, []);
    redirectMap[method] = finder.redirect;
    if (finder.usesArgumentsArray) {
      usesArgumentsArray.add(method);
    }
    return finder.redirect;
  }

  Method getTransitiveRedirect(Method method) {
    Method anchor = method; // For cycle detection.
    int reanchor = 4;
    Method next = getImmediateRedirect(method);
    while (next != null) {
      method = next;
      next = getImmediateRedirect(method);

      // Detect cycle. Cycles are not there in practice, but just in case.
      if (next == anchor) return anchor;
      if (--reanchor == 0) {
        anchor = method;
        reanchor *= 2;
      }
    }
    return method;
  }

  bool isRedirectTo(Method method, Obj function) {
    while (method != null) {
      if (method.function == function) return true;
      method = getImmediateRedirect(method);
    }
    return false;
  }

  bool targetUsesArgumentsArray(Method method) {
    return usesArgumentsArray.contains(getTransitiveRedirect(method));
  }

}

/// Analyzes the AST of a method to determine if it redirects to another function using `x.apply(_, arguments)`.
class RedirectFinder extends OnePassAbstractEvaluator {

  Method redirect;
  bool usesArgumentsArray = false;

  RedirectFinder(Snapshot snapshot) : super(snapshot);

  @override
  visitNameExpression(NameExpression node) {
    var value = super.visitNameExpression(node);
    if (value is ArgumentsArray) {
      usesArgumentsArray = true;
    }
    return value;
  }

  @override
  visitCall(CallExpression node) {
    Node callee = node.callee;
    if (callee is MemberExpression && callee.property.value == 'apply') {
      var target = visit(callee.object);
      var arguments = node.arguments.map(visit).toList();
      if (target is! Obj) return const NoValue();
      if (target.function is! UserFunc) return const NoValue();
      if (arguments.length == 2 && arguments[1] is ArgumentsArray) {
        redirect = new Method(target, arguments[0]);
      }
      return const NoValue();
    }
    return super.visitCall(node);
  }

}

String getLibraryNameFromFilename(String filename) {
  int slash = filename.lastIndexOf('/');
  if (slash != -1) filename = filename.substring(slash + 1);
  int dot = filename.indexOf('.');
  if (dot != -1) filename = filename.substring(0, dot);
  return filename;
}
