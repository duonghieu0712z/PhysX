/**
 * Makes the API a little less verbose
 */
Object.defineProperty(Module, 'PHYSICS_VERSION', {
  get() {
    return Module.TopLevelFunctions.prototype.PHYSICS_VERSION;
  },
});

// Move TopLevelFunctions to PhysX object
for (const prop in Module.TopLevelFunctions.prototype) {
  if (prop !== 'constructor' && !prop.startsWith('get_') && !prop.startsWith('__')) {
    Object.defineProperty(Module, prop, {
      get() {
        return Module.TopLevelFunctions.prototype[prop];
      },
    });
  }
}

// Move ExtensionFunctions to PhysX object
for (const prop in Module.ExtensionFunctions.prototype) {
  if (prop !== 'constructor' && !prop.startsWith('get_') && !prop.startsWith('__')) {
    Object.defineProperty(Module, prop, {
      get() {
        return Module.ExtensionFunctions.prototype[prop];
      },
    });
  }
}

// Move ArrayHelpers to PhysX object
for (const prop in Module.ArrayHelpers.prototype) {
  if (prop !== 'constructor' && !prop.startsWith('get_') && !prop.startsWith('__')) {
    Object.defineProperty(Module, prop, {
      get() {
        return Module.ArrayHelpers.prototype[prop];
      },
    });
  }
}

Module['malloc'] = Module['_webidl_malloc'] = _webidl_malloc;
Module['free'] = Module['_webidl_free'] = _webidl_free;

function deleteCache(obj) {
  delete Module.getCache(obj.__class__)[obj.ptr];
}
Module['deleteCache'] = deleteCache;

function release(obj) {
  if (!obj.release) return Module.destroy(obj);
  obj.release();
  deleteCache(obj);
}
Module['release'] = release;
