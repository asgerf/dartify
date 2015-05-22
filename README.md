# dartify

Generate Dart-JS interop classes from a JS file.

This is an experimental tool to see how much information we can extract out of a raw JS library.
(For class-based JS libraries, it's looking pretty good so far)

# Installation

	git clone https://github.com/asgerf/dartify
	cd dartify
	pub get
	npm install

I suggest you add the following script to your PATH:

	dart DARTIFY/bin/main.dart $*

# Usage

	dartify FILE.js > FILE.dart

Open `FILE.dart` to see your generated API. Import from Dart and have fun!

# Examples

See the `testcases` for some example libraries.

For example see [fabricjs.dart](testcases/fabricjs/fabricjs.dart) and the [example code](testcases/fabricjs/main.dart).

# Runtime System

dartify's runtime system for JS interop is based on dart:js and is only a hack right now.
It should ultimately integrate more closely with dart2js, dart:js, or package:js.

Currently, values returned from dart:js are not wrapped in typed proxies. That needs to be fixed.

# Implementation

dartify combines heap analysis and static analysis on the AST. 

The library is executed in a headless WebKit environment, and a heap snapshot is taken at the end of the top-level code.
At this point, all the classes are usually created and can be found in the heap.

Some tricks that are currently implemented:

- Names of parameters are taken from the AST.
- Detect redirect using `fun.apply(thisArg, arguments)`. Get parameter names from the effective target.
- Detect var-args if arguments array is used in a non-redirecting way.

Some things on the TODO list:

- Find instance fields.
- Find top-level methods (i.e. things are not just static on a class)
- Find static and top-level variables.
- Infer optional parameters (maybe even names arguments).
- Infer types for parameters and return types in places where it's easy.

