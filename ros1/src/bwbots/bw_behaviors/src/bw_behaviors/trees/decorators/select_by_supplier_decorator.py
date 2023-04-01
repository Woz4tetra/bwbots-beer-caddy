from typing import Callable, Dict
import py_trees


class SelectBySupplierDecorator(py_trees.decorators.Decorator):
    def __init__(self, children: Dict, name_supplier: Callable[[], str]):
        self.children_lookup = children
        self.name_supplier = name_supplier
        super().__init__(list(self.children_lookup.values())[0], py_trees.common.Name.AUTO_GENERATED)

        for child in self.children_lookup.values():
            child.parent = self
    
    def initialise(self):
        name = self.name_supplier()
        child = self.children_lookup[name]
        self.children[0] = child
        self.decorated = child

        return super().initialise()
    
    def update(self):
        return self.decorated.status
