library model;

class Program {
  String libraryName;
  List<Class> classes = <Class>[];
}

class Class {
  Class superClass; // May be null.
  List<String> jsConstructor; // Path in heap snapshot at which the JS constructor can be found.
  String name;
  Parameters constructorParams;
  List<Method> instanceMethods = <Method>[];
  List<Method> staticMethods = <Method>[];
}

class Method {
  String name;
  Parameters parameters;

  String get paramString => parameters.paramString;

  Method(this.name, this.parameters);
}

class Parameters {
  List<String> names;
  bool isVarArgs;

  String get paramString => names.join(', ');

  Parameters(this.names, {this.isVarArgs : false});
}
