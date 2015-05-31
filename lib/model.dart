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
  List<Field> instanceFields = <Field>[];
}

abstract class Member {
  String get name;
}

class Method implements Member {
  String name;
  Parameters parameters;

  String get paramString => parameters.paramString;

  Method(this.name, this.parameters);
}

class Field implements Member {
  String name;

  Field(this.name);
}

class Parameters {
  List<String> names;
  bool isVarArgs;

  String get paramString => names.join(', ');

  Parameters(this.names, {this.isVarArgs : false});
}
