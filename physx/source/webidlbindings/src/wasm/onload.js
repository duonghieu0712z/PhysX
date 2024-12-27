const PHYSICS_VERSION = 'PHYSICS_VERSION';
Object.defineProperty(Module, PHYSICS_VERSION, {
  get() {
    return Module['TopLevelFunctions'].prototype[PHYSICS_VERSION];
  },
});

const classes = ['TopLevelFunctions', 'ExtensionFunctions', 'ArrayHelpers'];
for (const name of classes) {
  const props = Module[name].prototype;
  for (const prop in props) {
    if (
      prop !== 'constructor' &&
      props[prop] !== props['constructor'] &&
      props[prop] !== props[`get_${PHYSICS_VERSION}`] &&
      !prop.startsWith('get_') &&
      !prop.startsWith('__')
    ) {
      Object.defineProperty(Module, prop, {
        get() {
          return props[prop];
        },
      });
    }
  }
}

Module['malloc'] = Module['_webidl_malloc'] = _webidl_malloc;
Module['free'] = Module['_webidl_free'] = _webidl_free;

function deleteCache(obj, recursive = false) {
  const caches = Module['getCache'](Module['getClass'](obj));
  const ptr = Module['getPointer'](obj);
  if (!caches[ptr]) {
    return;
  }
  delete caches[ptr];

  if (recursive) {
    for (const prop in obj) {
      if (prop.startsWith('get_') && prop !== 'get_userData') {
        const value = obj[prop]();
        if (typeof value === 'object') {
          deleteCache(value, true);
        }
      }
    }
  }
}
Module['deleteCache'] = deleteCache;

function release(obj) {
  if (!obj['release']) {
    Module['destroy'](obj);
    return;
  }

  obj['release']();
  Module['deleteCache'](obj);
}
Module['release'] = release;
