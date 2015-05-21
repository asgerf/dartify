library model;

class Program {
  String libraryName;
  List<Class> classes = <Class>[];
}

class Class {
  Class superClass; // May be null.
  List<String> jsConstructor; // Path in heap snapshot at which the JS constructor can be found.
  String name;
  List<String> constructorParams;
  List<Method> instanceMethods = <Method>[];
  List<Method> staticMethods = <Method>[];
}

class Method {
  String name;
  List<String> parameters;

  String get paramString => parameters.join(', ');

  Method(this.name, this.parameters);
}
