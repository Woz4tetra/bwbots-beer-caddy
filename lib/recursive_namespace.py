from types import SimpleNamespace


class RecursiveNamespace(SimpleNamespace):
    """
    Similar to types.SimpleNamespace, except nested dictionaries become RecursiveNamespace too
    Allows for reference by dot and array notation:
    ns["something"] == ns.something
    """

    @staticmethod
    def map_entry(entry):
        if isinstance(entry, dict):
            return RecursiveNamespace(**entry)
        return entry

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.update(kwargs)

    def to_dict(self) -> dict:
        """Recursively convert RecursiveNamespace to dict"""
        conversion = dict()
        self._to_dict_recurse(conversion)
        return conversion

    def _to_dict_recurse(self, conversion: dict):
        """to_dict is the wrapper. This is the function that recurses"""
        for attr_name, attr_val in self.__dict__.items():
            if isinstance(attr_val, RecursiveNamespace):
                # if an attribute in the top level in a RecursiveNamespace, recurse into
                # that object and convert it to a dictionary
                conversion[attr_name] = attr_val.to_dict()
            elif type(attr_val) == list or type(attr_val) == tuple:
                # if an attribute is a list or tuple, iterate through it and make sure all
                # RecursiveNamespace's are converted to dict
                result = []
                for list_attr in attr_val:
                    if isinstance(list_attr, RecursiveNamespace):
                        result.append(list_attr.to_dict())
                    else:
                        result.append(list_attr)
                conversion[attr_name] = result
            else:
                # otherwise, add attribute to dict
                conversion[attr_name] = attr_val

    def get_nested(self, keys: tuple):
        """
        Access a nested namespace by tuple.
            ex. ns.get_nested(("A", "A_C", 0, "A_C_A")) == 2.0
        The dictionarified version of this namespace is:
            ns.to_dict() == {
                'A': {
                    'A_C': [
                        {'A_C_A': 2.0}
                    ]
                }
            }
        If any keys provided don't exist, this raises AttributeError
        :param keys: tuple of keys. Indices of increasing length indicate an nested level of the namespace
        :return: the value contained in the nested namespace
        """
        return self._get_nested_recurse(self, list(keys))

    def get_nested_default(self, keys: tuple, default=None):
        """Same as get_nested except if the any of keys don't exist, it returns the default value provided."""
        try:
            return self.get_nested(keys)
        except AttributeError:
            return default

    def _get_nested_recurse(self, tree, keys: list):
        """get_nested is the wrapper. This is the recursing function"""
        if len(keys) == 0:  # if there are no more keys, return the current value we're at
            return tree
        key = keys.pop(0)  # remove the first key an access the tree with it
        value = tree[key]  # pull the next value
        if isinstance(value, RecursiveNamespace) or type(value) == list or type(value) == tuple:
            # if it's a list or namespace, we need to go a level deeper.
            # we've already checked if we run out of keys and we haven't at this point
            return self._get_nested_recurse(value, keys)
        else:
            # we've reached the bottom but there are still keys that weren't used.
            # throw an error
            if len(keys) != 0:
                raise AttributeError("Supplied sub keys are not accounted for: " + str(keys))

            # we've reached the bottom but we've run out of keys. Return the value we found
            return value

    def set_nested(self, keys: tuple, value, create=False):
        """
        Set a nested namespace by tuple.
            ex. say we start with a RecursiveNamespace:
            ns = RecursiveNamespace(**{
                'A': {
                    'A_C': [
                        {'A_C_A': 2.0}
                    ]
                }
            })
            ns.set_nested(("A", "A_C", 0, "A_C_A"), 3.0)

            The dictionarified version of this namespace is now:
            ns.to_dict() == {
                'A': {
                    'A_C': [
                        {'A_C_A': 3.0}
                    ]
                }
            }

        It's possible to create nested namespaces with this method if create is set to True.
        If we start with a blank namespace and call:
        ns.set_nested(("A", "A_C", "A_C_A"), 3.0)
        The dictionarified version of this namespace is now:
        ns.to_dict() == {
            'A': {
                'A_C': {'A_C_A': 3.0}
            }
        }
        Note, it's not possible to create nested lists with this method.
        Check out the merge method if you're combining namespaces.
        If create is False, an AttributeError is raised if sub-keys don't exist

        :param keys: tuple of keys to use when setting the namespace
        :param value: value to set namespace with
        :param create: whether to create nested RecursiveNamespace's if sub keys don't exist
        :return: None
        """
        if isinstance(value, dict):
            value = RecursiveNamespace(**value)
        self._set_nested_recurse(self, list(keys), value, create)

    def _set_nested_recurse(self, tree, keys: list, value, create):
        """set_nested is the wrapper. _set_nested_recurse is the recursing function"""
        key = keys.pop(0)
        if len(keys) == 0:  # if there are no more keys, the namespace level we're at with the desired value
            tree[key] = value
            return

        if create and key not in tree.__dict__.keys():
            # If create is True and the sub-key doesn't exist, create a new RecursiveNamespace and recurse into it
            tree[key] = RecursiveNamespace()
        next_tree = tree[key]  # get the next namespace or list to recurse into
        if isinstance(next_tree, RecursiveNamespace) or type(next_tree) == list or type(next_tree) == tuple:
            return self._set_nested_recurse(next_tree, keys, value, create)
        elif len(keys) != 0:
            raise AttributeError("Supplied sub keys are not accounted for: " + str(keys))

    def get(self, key, default=None):
        """
        Similar to dict's get method. Access's top level of RecursiveNamespace and returns the default
        value if the key doesn't exist.
        :param key: key to use when accessing the top level
        :param default: value to return if the key doesn't exist
        :return: value in namespace key
        """
        if key in self.keys():
            return self[key]
        else:
            return default

    def flatten(self, stringify_separator=None) -> dict:
        """
        Flatten namespace into a dictionary where the keys are tuple keys representing the namespace structure.
        These keys are exactly the same format used for get_nested and set_nested.
        ex.
        ns = RecursiveNamespace(**{
            "A": {"A_A": 0, "A_B": 1, "A_C": [{"A_C_A": 2.0}, 2.1, 2.2]},
            "B": {"B_A": 0, "B_B": 1, "B_C": [{"B_C_A": 2.0}, 2.1, 2.2], "B_D": 3},
        })
        ns.flatten() == {
            ('A', 'A_A'): 0,
            ('A', 'A_B'): 1,
            ('A', 'A_C', 0, 'A_C_A'): 2.0,
            ('A', 'A_C', 1): 2.1,
            ('A', 'A_C', 2): 2.2,
            ('B', 'B_A'): 0,
            ('B', 'B_B'): 1,
            ('B', 'B_C', 0, 'B_C_A'): 2.0,
            ('B', 'B_C', 1): 2.1,
            ('B', 'B_C', 2): 2.2,
            ('B', 'B_D'): 3
        }

        If stringify_separator is set to "/" (not None), the tuples are replaced with a delimited string:
       ns.flatten(stringify_separator="/") == {
            'A/A_A': 0,
            'A/A_B': 1,
            'A/A_C/0/A_C_A': 2.0,
            'A/A_C/1': 2.1,
            'A/A_C/2': 2.2,
            'B/B_A': 0,
            'B/B_B': 1,
            'B/B_C/0/B_C_A': 2.0,
            'B/B_C/1': 2.1,
            'B/B_C/2': 2.2,
            'B/B_D': 3
        }

        :param stringify_separator: If None, tuple keys are used, otherwise delimited strings are used
        :return: dict, flattened namespace
        """
        flat = {}
        self._flatten_recurse([], flat, self.__dict__)
        if stringify_separator is not None:
            assert type(stringify_separator) == str
            stringify_flat = {}
            for key in flat.keys():
                str_key = "/".join(list(map(str, key)))
                stringify_flat[str_key] = flat[key]
            return stringify_flat
        else:
            return flat

    def _flatten_recurse(self, root_key, flat, tree: dict):
        """flatten is the wrapper. _flatten_recurse is the recursive function"""
        for attr_name, attr_val in tree.items():
            flat_key = root_key + [attr_name]
            if isinstance(attr_val, RecursiveNamespace):
                self._flatten_recurse(flat_key, flat, attr_val.__dict__)
            elif type(attr_val) == list or type(attr_val) == tuple:
                for index, list_attr in enumerate(attr_val):
                    list_key = flat_key + [index]
                    if isinstance(list_attr, RecursiveNamespace):
                        self._flatten_recurse(list_key, flat, list_attr.__dict__)
                    else:
                        flat[tuple(list_key)] = list_attr
            else:
                flat[tuple(flat_key)] = attr_val

    def items(self):
        """Similar to dict.items(). Gets top level keys and values as a iterable"""
        return self.__dict__.items()

    def keys(self):
        """Similar to dict.keys(). Gets top level keys as a iterable"""
        return self.__dict__.keys()

    def values(self):
        """Similar to dict.values(). Gets top level values as a iterable"""
        return self.__dict__.values()

    def update(self, d: dict):
        """
        Similar to dict.update(). Applies all values in supplied dictionary to namespace.
        All nested dictionaries are converted into RecursiveNamespace

        ex.
        ns = RecursiveNamespace(**{
            "A": {"A_A": 0, "A_B": 1, "A_C": [{"A_C_A": 2.0}, 2.1, 2.2]},
            "B": {"B_A": 0, "B_B": 1, "B_C": [{"B_C_A": 2.0}, 2.1, 2.2], "B_D": 3},
        })
        overlay = RecursiveNamespace(**{
            "A": {"A_C": 4},
            "B": {"B_C": [5.0], "B_D": 6},
            "C": {"C_D": 7},
        })

        ns.update(overlay)
        ns.to_dict() == {
            'A': {'A_C': 4},
            'B': {'B_C': [5.0], 'B_D': 6},
            'C': {'C_D': 7}
        }
        """

        for key, val in d.items():
            if isinstance(val, dict):
                setattr(self, key, RecursiveNamespace(**val))
            elif type(val) == list or type(val) == tuple:
                setattr(self, key, list(map(self.map_entry, val)))
            else:
                setattr(self, key, val)

    def merge(self, other):
        """
        A more sophisticated version of upate. If keys match at a certain level in the two namespaces,
        this method will recurse further and check if any more levels match.

        ex.
        ns = RecursiveNamespace(**{
            "A": {"A_A": 0, "A_B": 1, "A_C": [{"A_C_A": 2.0}, 2.1, 2.2]},
            "B": {"B_A": 0, "B_B": 1, "B_C": [{"B_C_A": 2.0}, 2.1, 2.2], "B_D": 3},
        })
        overlay = RecursiveNamespace(**{
            "A": {"A_C": 4},
            "B": {"B_C": [5.0], "B_D": 6},
            "C": {"C_D": 7},
        })

        ns.merge(overlay)
        ns.to_dict() == {
            'A': {'A_A': 0, 'A_B': 1, 'A_C': 4},
            'B': {'B_A': 0, 'B_B': 1, 'B_C': [5.0], 'B_D': 6},
            'C': {'C_D': 7}
        }

        Compare this with the update method example. update replaced all the top level keys with its values
        since they matched. In merge, the nested namespaces are combined if the keys match.
        This is useful for config files with a base and overlay configuration. You want to preserve the base keys
        and overwrite the overlay keys where defined.

        :param other: RecursiveNamespace to merge with
        :return: None
        """
        if not isinstance(other, RecursiveNamespace):
            raise AttributeError("Merging object is not RecursiveNamespace: <%s>%s" % (type(other), repr(other)))
        for key, value in other.items():
            if key in self.keys() and \
                    isinstance(value, RecursiveNamespace) and \
                    isinstance(self[key], RecursiveNamespace):
                self[key].merge(value)
            else:
                self[key] = value

    def __getitem__(self, key):
        """Access an item at the top level using array get notation"""
        return getattr(self, key)

    def __setitem__(self, key, value):
        """Set an item at the top level using array set notation"""
        if isinstance(value, dict):
            value = self.__class__(**value)
        return setattr(self, key, value)

    def __eq__(self, other: object) -> bool:
        """Check if another object is equivalent to this namespace. Values are checked if dict or RecursiveNamespace"""
        if isinstance(other, dict):
            return self.to_dict() == other
        elif isinstance(other, RecursiveNamespace):
            return self.to_dict() == other.to_dict()
        else:
            return False


if __name__ == '__main__':
    import pprint


    def test():
        test_dict = {
            "A": {"A_A": 0, "A_B": 1, "A_C": [{"A_C_A": 2.0}, 2.1, 2.2]},
            "B": {"B_A": 0, "B_B": 1, "B_C": [{"B_C_A": 2.0}, 2.1, 2.2], "B_D": 3},
            "C": {"C_A": 0, "C_B": 1, "C_C": [{"C_C_A": 2.0}, 2.1, 2.2], "C_D": 3, "C_E": 4, "C_F": "567"},
        }
        ns = RecursiveNamespace(**test_dict)
        pprint.pprint(test_dict)
        pprint.pprint(ns.to_dict())
        assert ns.to_dict() == test_dict, "%s != %s" % (ns.to_dict(), test_dict)
        assert ns.A.A_A == 0
        assert ns.A.A_C[0].A_C_A == 2.0
        assert ns.A.A_C[1] == 2.1
        assert ns.B.B_B == 1
        assert ns.C.C_E == 4
        assert ns["A"]["A_A"] == 0
        assert ns["A"]["A_C"][0]["A_C_A"] == 2.0
        assert ns["A"]["A_C"][1] == 2.1
        assert ns["B"]["B_B"] == 1
        assert ns["C"]["C_E"] == 4
        assert ns["C"]["C_F"] == "567"

        assert ns.get_nested(("A", "A_A")) == 0
        assert ns.get_nested(("A", "A_C", 0, "A_C_A")) == 2.0
        assert ns.get_nested_default(("A", "A_C", 0, "A_C_B"), None) == None
        try:
            ns.get_nested(("A", "A_C", 0, "A_C_A", "something not here", "something else not here"))
        except AttributeError:
            assert True
        else:
            assert False

        test_flat = ns.flatten()
        true_flat = {
            ("A", "A_A"): 0,
            ("A", "A_B"): 1,
            ("A", "A_C", 0, "A_C_A"): 2.0,
            ("A", "A_C", 1): 2.1,
            ("A", "A_C", 2): 2.2,
            ("B", "B_A"): 0,
            ("B", "B_B"): 1,
            ("B", "B_C", 0, "B_C_A"): 2.0,
            ("B", "B_C", 1): 2.1,
            ("B", "B_C", 2): 2.2,
            ("B", "B_D"): 3,
            ("C", "C_A"): 0,
            ("C", "C_B"): 1,
            ("C", "C_C", 0, "C_C_A"): 2.0,
            ("C", "C_C", 1): 2.1,
            ("C", "C_C", 2): 2.2,
            ("C", "C_D"): 3,
            ("C", "C_E"): 4,
            ("C", "C_F"): "567",
        }
        assert test_flat == true_flat
        for key, value in test_flat.items():
            test_value = ns.get_nested(key)
            assert test_value == value, "%s != %s" % (value, test_value)

        stringify_truth = {
            "A/A_A": 0,
            "A/A_B": 1,
            "A/A_C/0/A_C_A": 2.0,
            "A/A_C/1": 2.1,
            "A/A_C/2": 2.2,
            "B/B_A": 0,
            "B/B_B": 1,
            "B/B_C/0/B_C_A": 2.0,
            "B/B_C/1": 2.1,
            "B/B_C/2": 2.2,
            "B/B_D": 3,
            "C/C_A": 0,
            "C/C_B": 1,
            "C/C_C/0/C_C_A": 2.0,
            "C/C_C/1": 2.1,
            "C/C_C/2": 2.2,
            "C/C_D": 3,
            "C/C_E": 4,
            "C/C_F": "567",
        }
        test_stringify = ns.flatten(stringify_separator="/")
        assert len(stringify_truth) == len(test_stringify)
        for key in stringify_truth.keys():
            assert stringify_truth[key] == test_stringify[key]

        overlay_dict = {
            "A": {"A_C": 4},
            "B": {"B_C": [5.0], "B_D": 6},
            "C": {"C_D": 7},
        }
        base_ns = RecursiveNamespace(**ns.to_dict())
        overlay_ns = RecursiveNamespace(**overlay_dict)
        base_ns.merge(overlay_ns)
        test_merge_flat = base_ns.flatten()
        true_merge_flat = {
            ("A", "A_A"): 0,
            ("A", "A_B"): 1,
            ('A', 'A_C'): 4,
            ("B", "B_A"): 0,
            ("B", "B_B"): 1,
            ("B", "B_C", 0): 5.0,
            ("B", "B_D"): 6,
            ("C", "C_A"): 0,
            ("C", "C_B"): 1,
            ("C", "C_C", 0, "C_C_A"): 2.0,
            ("C", "C_C", 1): 2.1,
            ("C", "C_C", 2): 2.2,
            ("C", "C_D"): 7,
            ("C", "C_E"): 4,
            ("C", "C_F"): "567",
        }
        assert test_merge_flat == true_merge_flat

        modify_ns = RecursiveNamespace(**ns.to_dict())
        true_modify_ns = RecursiveNamespace(**{
            "A": {"A_A": 101, "A_B": 1, "A_C": [{"A_C_A": 2.0}, 2.1, 2.2]},
            "B": {"B_A": 0, "B_B": 1, "B_C": [{"B_C_A": 2.0}, 2.1, {"B_C_B": 2.2, "B_C_C": 2.3}], "B_D": 3},
            "C": {"C_A": 0, "C_B": 1, "C_C": [{"C_C_A": 2.0}, 2.1, 2.2], "C_D": 3, "C_E": 4, "C_F": "567"},
        })
        modify_ns.set_nested(("A", "A_A"), 101)
        modify_ns.set_nested(("B", "B_C", 2), {"B_C_B": 2.2, "B_C_C": 2.3})

        assert modify_ns == true_modify_ns

        modify_ns.set_nested(("A", "A_A"), 100)
        assert modify_ns != true_modify_ns
        modify_ns.set_nested(("B", "B_C", 2, "B_C_B"), 220)
        assert modify_ns != true_modify_ns
        modify_ns.set_nested(("A", "A_A"), 101)
        modify_ns.set_nested(("B", "B_C", 2, "B_C_B"), 2.2)
        assert modify_ns == true_modify_ns

        try:
            modify_ns.set_nested(("D", "D_A", "D_A_A", "D_A_A_A", "D_A_A_A_A"), "E")
            assert False
        except AttributeError:
            assert True
        modify_ns.set_nested(("D", "D_A", "D_A_A", "D_A_A_A", "D_A_A_A_A"), "E", create=True)
        assert modify_ns.get_nested(("D", "D_A", "D_A_A", "D_A_A_A", "D_A_A_A_A")) == "E"

        print("All tests pass!")


    test()
