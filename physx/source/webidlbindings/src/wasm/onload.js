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
