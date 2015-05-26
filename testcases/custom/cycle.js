function Cycle() {
}
Cycle.prototype.chicken = function() {
	return this.egg.apply(this, arguments);
}
Cycle.prototype.egg = function() {
	return this.chicken.apply(this, arguments);
}
