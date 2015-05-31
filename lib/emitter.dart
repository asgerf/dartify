import 'model.dart';


final Set<String> UnsafeClassNames = new Set.from(['Object']); // TODO
String getClassName(Class clazz) {
  String name = clazz.name;
  if (UnsafeClassNames.contains(name)) return name + r'$';
  return name;
}

final Set<String> UnsafeMemberNames = new Set.from([]);
String getMemberName(Member member) {
  String name = member.name;
  if (UnsafeMemberNames.contains(name)) return name + r'$';
  return name;
}

String getClassProxyName(Class clazz) {
  return '_' + clazz.name;
}

bool hasPrefix(String name, String prefix) {
  return name.length > prefix.length && name.startsWith(prefix) && name[prefix.length] == name[prefix.length].toUpperCase();
}

String flopPrefix(String name, String prefix) {
  return name[prefix.length].toLowerCase() + name.substring(prefix.length) + prefix;
}

String getSortKey(Method method) {
  String name = method.name;
  if (hasPrefix(name, 'get')) return flopPrefix(name, 'get');
  if (hasPrefix(name, 'set')) return flopPrefix(name, 'set');
  if (hasPrefix(name, 'is')) return flopPrefix(name, 'is');
  return name;
}

compareBy(Comparable func(x)) => (x,y) {
  return func(x).compareTo(func(y));
};

void emit(Program program) {
  print("library ${program.libraryName};\n");
  print("import 'dart:js';\n");

  // Figure out which classes have subclasses.
  Set<Class> classesWithSubclasses = new Set<Class>();
  for (Class clazz in program.classes) {
    if (clazz.superClass != null) classesWithSubclasses.add(clazz.superClass);
  }

  // Sort methods in each class
  for (Class clazz in program.classes) {
    clazz.instanceMethods.sort(compareBy(getSortKey));
    clazz.staticMethods.sort(compareBy(getSortKey));
  }

  // Print beautiful interfaces
  for (Class clazz in program.classes) {
    String className = getClassName(clazz);
    String proxyName = getClassProxyName(clazz);

    String extendsClause = '';
    if (clazz.superClass != null) {
      String superClassName = getClassName(clazz.superClass);
      extendsClause = 'extends $superClassName ';
    }
    print('abstract class $className $extendsClause{');

    // Print constructor.
    if (clazz.constructorParams.isVarArgs) {
      print('  factory $className(List args) = _${clazz.name};');
    } else {
      print('  factory $className(${clazz.constructorParams.paramString}) = _${clazz.name};');
    }

    print(''); // Make some space.

    for (Field field in clazz.instanceFields) {
      String name = getMemberName(field);
      print('  var $name;');
    }
    if (clazz.instanceFields.isNotEmpty && clazz.instanceMethods.isNotEmpty) {
      print('');
    }
    for (Method method in clazz.instanceMethods) {
      String name = getMemberName(method);
      if (method.parameters.isVarArgs) {
        print('  $name(List args);');
      } else {
        print('  $name(${method.paramString});');
      }
    }
    if (clazz.instanceMethods.isNotEmpty && clazz.staticMethods.isNotEmpty) {
      print('');
    }
    for (Method method in clazz.staticMethods) {
      String name = getMemberName(method);
      if (method.parameters.isVarArgs) {
        print('  static $name(List args) => $proxyName.$name(args);');
      } else {
        print('  static $name(${method.paramString}) => $proxyName.$name(${method.paramString});');
      }
    }

    print('}\n');
  }

  // Print the superclass of the proxies.
  print('''
  abstract class _Proxy {
    get jsvalue;
  }

  _(x) {
    if (x is _Proxy) return x.jsvalue;
    if (x is Map || x is Iterable) return new JsObject.jsify(x);
    return x;
  }
''');

  unfold(x) => '_($x)';

  // Print ugly proxy classes
  for (Class clazz in program.classes) {
    String className = getClassName(clazz);
    String proxyName = getClassProxyName(clazz);
    bool hasSubclass = classesWithSubclasses.contains(clazz);

    // Extend the proxy for the super class, if any
    String extendsClause;
    if (clazz.superClass != null) {
      String superClassName = getClassProxyName(clazz.superClass);
      extendsClause = 'extends $superClassName ';
    } else {
      extendsClause = 'extends _Proxy ';
    }
    extendsClause += 'implements $className';
    print('class $proxyName $extendsClause {');

    // A non-subclassed proxy declares a field _jsvalue pointing to the JsObject.
    if (clazz.superClass == null) {
      print('  var jsvalue;');
    }

    // Print reference to the constructor as a JsObject
    String dartGlobalExpr = 'context';
    for (String pathItem in clazz.jsConstructor) {
      dartGlobalExpr += '["' + pathItem + '"]';
    }
    print('  static JsObject _constructor = $dartGlobalExpr;');

    // Returns a constructor initializer that ensures that the jsvalue field gets initialized.
    String initValue(String value) {
      if (clazz.superClass == null) {
        return 'jsvalue = $value';
      } else {
        return 'super.withValue($value)';
      }
    }

    // Print the main constructor.
    String constructorParams = clazz.constructorParams.isVarArgs ? 'args' : clazz.constructorParams.paramString;
    String arguments = clazz.constructorParams.isVarArgs ? 'args' : '[${clazz.constructorParams.names.map(unfold).join(', ')}]';
    print('  $proxyName($constructorParams) : ' + initValue('new JsObject(_constructor, $arguments)') + ';');

    // Print the withValue constructor if there is a subclass who needs it.
    if (hasSubclass) {
      print('  $proxyName.withValue(value) : ' + initValue('value') + ';');
    }

    print(''); // Make some space before the methods.

    for (Field field in clazz.instanceFields) {
      String name = getMemberName(field);
      String jsname = field.name;
      print('  get $name => jsvalue["$jsname"];');
      print('  set $name(x) => jsvalue["$jsname"] = x;');
    }
    for (Method method in clazz.instanceMethods) {
      String name = getMemberName(method);
      String jsname = method.name;
      if (method.parameters.isVarArgs) {
        print('  $name(args) => jsvalue.callMethod(r"$jsname", args.map(_).toList());');
      } else {
        String arguments = method.parameters.names.map(unfold).join(', ');
        print('  $name(${method.paramString}) => jsvalue.callMethod(r"$jsname", [$arguments]);');
      }
    }
    for (Method method in clazz.staticMethods) {
      String name = getMemberName(method);
      String jsname = method.name;
      if (method.parameters.isVarArgs) {
        print('  static $name(args) => _constructor.callMethod(r"$jsname", args.map(_).toList());');
      } else {
        String arguments = method.parameters.names.map(unfold).join(', ');
        print('  static $name(${method.paramString}) => _constructor.callMethod(r"$jsname", [$arguments]);');
      }
    }

    print('}\n');
  }

}
