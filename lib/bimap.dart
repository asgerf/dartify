library util;

/// Bijective map; maintains a 1:1 mapping between elements from different domains.
///
/// Bimap contains a map from K to V and a map from V to K. It maintains
/// the invariant that there is always a 1:1 mapping between K and V elements.
/// For example:
///
///     Bimap<int, String> map = new Bimap<int, String>();
///
///     map[1] = 'Hello';
///     map[2] = 'World';
///     print(map[1]); // Prints 'Hello'
///     print(map[2]); // Prints 'World'
///     print(map.inverse['Hello']); // Prints '1'
///     print(map.inverse['World']); // Prints '2'
///
/// It is an error to overwrite an existing key-value binding without removing
/// the old bindings first. In checked-mode, this generates an assertion failure:
///
///     map[3] = 'Hello';     // Assertion error: Value 'Hello' is already in the map.
///     map[1] = 'Greetings'; // Assertion error: Key 1 is already in the map.
///
class Bimap<K, V> {
  final Map<K, V> _forward;
  final Map<V, K> _backward;

  Bimap() : _forward = <K, V>{}, _backward = <V, K>{};

  Bimap._internal(this._forward, this._backward);

  /// The inverse view of this map. Use this for V-to-K queries.
  ///
  /// A new object is created for every invocation; so avoid long chains of .inverse.inverse.inverse calls.
  Bimap<V, K> get inverse => new Bimap<V, K>._internal(_backward, _forward);

  Map<K, V> get asMap => _forward;
  Map<V, K> get asInverseMap => _backward;

  /**
   * Returns true if this map contains the given [value].
   *
   * Returns true if any of the values in the map are equal to `value`
   * according to the `==` operator.
   */
  bool containsValue(Object value) {
    return _backward.containsKey(value);
  }

  /**
   * Returns true if this map contains the given [key].
   *
   * Returns true if any of the keys in the map ar equal to `key`
   * according to the equality used by the map.
   */
  bool containsKey(Object key) {
    return _forward.containsKey(key);
  }

  /**
   * Returns the value for the given [key] or null if [key] is not in the map.
   *
   * Some maps allows keys to have `null` as a value,
   * For those maps, a lookup using this operator does cannot be used to
   * distinguish between a key not being in the map, and the key having a null
   * value.
   * Methods like [containsKey] or [putIfAbsent] can be use if the distinction
   * is important.
   */
  V operator [](Object key) {
    return _forward[key];
  }

  /**
   * Associates the [key] with the given [value].
   *
   * It is an error to invoke this if [key] is already associated with a value,
   * or if [value] is already associated with a key.
   */
  void operator []=(K key, V value) {
    assert(!_forward.containsKey(key));
    assert(!_backward.containsKey(value));
    _forward[key] = value;
    _backward[value] = key;
  }

  /**
   * Look up the value of [key], or add a new value if it isn't there.
   *
   * Returns the value associated to [key], if there is one.
   * Otherwise calls [ifAbsent] to get a new value, associates [key] to
   * that value, and then returns the new value.
   *
   *     Map<String, int> scores = {'Bob': 36};
   *     for (var key in ['Bob', 'Rohan', 'Sophena']) {
   *       scores.putIfAbsent(key, () => key.length);
   *     }
   *     scores['Bob'];      // 36
   *     scores['Rohan'];    //  5
   *     scores['Sophena'];  //  7
   *
   * Calling [ifAbsent] must not add or remove keys from the map.
   *
   * It is an error for [ifAbsent] to return a value that is already associated
   * with a key.
   */
  V putIfAbsent(K key, V ifAbsent()) {
    return _forward.putIfAbsent(key, () {
      V value = ifAbsent();
      assert(!_backward.containsKey(value));
      _backward[value] = key;
      return value;
    });
  }

  /**
   * Adds all key-value pairs of [other] to this map.
   *
   * If a key of [other] is already in this map, its value is overwritten.
   *
   * The operation is equivalent to doing `this[key] = value` for each key
   * and associated value in other. It iterates over [other], which must
   * therefore not change during the iteration.
   */
  void addAll(Map<K, V> other) {
    other.forEach((K key, V value) {
      this[key] = value;
    });
  }

  /**
   * Removes [key] and its associated value, if present, from the map.
   *
   * Returns the value associated with `key` before it was removed.
   * Returns `null` if `key` was not in the map.
   *
   * Note that values can be `null` and a returned `null` value doesn't
   * always mean that the key was absent.
   */
  V remove(K key) {
    V oldValue = _forward.remove(key);
    if (oldValue != null) {
      _backward.remove(oldValue);
    }
    return oldValue;
  }

  /**
   * Removes [value] and its associated key, if present, from the map.
   *
   * Returns the key associated with `value` before it was removed.
   * Returns `null` if `value` was not in the map.
   *
   * Note that keys can be `null` and a returned `null` value doesn't
   * always mean that the value was absent.
   */
  K removeValue(V value) {
    K oldKey = _backward.remove(value);
    if (oldKey != null) {
      _forward.remove(oldKey);
    }
    return oldKey;
  }

  /**
   * Removes all pairs from the map.
   *
   * After this, the map is empty.
   */
  void clear() {
    _forward.clear();
    _backward.clear();
  }

  /**
   * Applies [f] to each key-value pair of the map.
   *
   * Calling `f` must not add or remove keys from the map.
   */
  void forEach(void f(K key, V value)) => _forward.forEach(f);

  /**
   * The keys of [this].
   *
   * The returned iterable has efficient `length` and `contains` operations,
   * based on [length] and [containsKey] of the map.
   *
   * The order of iteration is defined by the individual `Map` implementation,
   * but must be consistent between changes to the map.
   */
  Iterable<K> get keys => _forward.keys;

  /**
   * The values of [this].
   *
   * The values are iterated in the order of their corresponding keys.
   * This means that iterating [keys] and [values] in parrallel will
   * provided matching pairs of keys and values.
   *
   * The returned iterable has an efficient `length` method based on the
   * [length] of the map. Its [Iterable.contains] method is based on
   * `==` comparison.
   */
  Iterable<V> get values => _forward.values; // Must match iteration order with keys!

  /**
   * The number of key-value pairs in the map.
   */
  int get length => _forward.length;

  /**
   * Returns true if there is no key-value pair in the map.
   */
  bool get isEmpty => _forward.isEmpty;

  /**
   * Returns true if there is at least one key-value pair in the map.
   */
  bool get isNotEmpty => _forward.isNotEmpty;

}
