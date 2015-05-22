// Copyright (c) 2015, <your name>. All rights reserved. Use of this source code
// is governed by a BSD-style license that can be found in the LICENSE file.

import 'dart:io';

import 'package:dartify/dartify.dart';
import 'package:parsejs/parsejs.dart' as js;
import 'package:dartify/jsnap.dart' as jsnap;
import 'package:dartify/emitter.dart' as emitter;
import 'package:dartify/model.dart' as model;

abort(String msg) {
  stderr.writeln(msg);
  exit(1);
}

help([String msg]) {
  if (msg != null) {
    stderr..writeln(msg)..writeln('');
  }
  abort('''
Usage:

  dartify FILE.js > FILE.dart

First-time setup:

  pub get
  npm install
''');
}

String findJSnapCommand() {
  File localInstall = new File('../node_modules/jsnap/jsnap.js');
  if (localInstall.existsSync()) {
    return localInstall.path;
  } else {
    abort('jsnap not found. Please run `npm install`.');
  }
}

main(List<String> args) {
  if (args.isEmpty) help();
  File jsFile = new File(args[0]);
  File jsnapFile = new File(jsFile.path + 'nap');
  String jsnapText;

  // JS file must be there.
  if (!jsFile.existsSync()) abort("File not found: '${jsFile.path}'");

  // Regenerate jsnap file if absent or older than the JS file.
  if (!jsnapFile.existsSync() || jsnapFile.lastModifiedSync().isBefore(jsFile.lastModifiedSync())) {
    try {
      jsnapText = Process.runSync(findJSnapCommand(), [jsFile.path]).stdout;
    } on ProcessException catch (e) {
      abort('Error running `${e.executable} ${e.arguments.join(' ')}`.\n${e.message}');
    }
    jsnapFile.writeAsStringSync(jsnapText);
  } else {
    jsnapText = jsnapFile.readAsStringSync();
  }

  jsnap.Snapshot snapshot = jsnap.parseSnapshot(jsnapText);
  String jsText = jsFile.readAsStringSync();
  js.Program program = js.parsejs(jsText, filename: jsFile.path);
  
  model.Program output = dartify(snapshot, program);

  emitter.emit(output);
}
