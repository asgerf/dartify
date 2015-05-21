function StringBuilder() {
	this.strings = [];
}
StringBuilder.prototype.append = function(x) {
	this.strings.add(x);
}
StringBuilder.prototype.toString = function() {
	return this.strings.join('');
}


var foo = new StringBuilder();
   