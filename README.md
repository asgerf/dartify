# dartify

Generate Dart-JS interop classes from a JS file.

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
