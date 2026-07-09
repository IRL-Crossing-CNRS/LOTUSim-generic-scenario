from __future__ import annotations

from importlib.metadata import entry_points
from typing import Dict, Type

from lotusim_sdk.bt.composites import Parallel, Sequence
from lotusim_sdk.bt.node import BehaviorNode

# Composite node types addressable from the JSON "type" field.
# (Fallback / decorators are deferred to a later version.)
_COMPOSITES = {"sequence": Sequence, "parallel": Parallel}


def load_task_registry() -> Dict[str, Type]:
    """Map task name -> TaskAgent subclass, discovered across ALL
    installed wheels via the ``lotusim.tasks`` entry-point group.

    This mirrors the existing agent discovery (``lotusim.agents``): a task
    defined in a PhD's own wheel appears here automatically, with no edit to the
    core repo.
    """
    registry: Dict[str, Type] = {}
    for ep in entry_points(group="lotusim.tasks"):
        registry[ep.name] = ep.load()
    return registry


def build_tree(spec: dict, host, blackboard, registry: Dict[str, Type]) -> BehaviorNode:
    """Recursively turn one JSON mission node into a tree of ``BehaviorNode``.

    A composite receives its already-built children; a leaf is resolved through
    the registry to the right ``TaskAgent`` subclass and instantiated
    with its ``params`` and the shared ``host``/``blackboard``.
    """
    node_type = spec.get("type")

    if node_type in _COMPOSITES:
        children = [
            build_tree(child, host, blackboard, registry)
            for child in spec.get("children", [])
        ]
        kwargs = {}
        if node_type == "parallel":
            kwargs["success_policy"] = spec.get("success_policy", "all")
        return _COMPOSITES[node_type](id=spec.get("id", ""), children=children, **kwargs)

    # Leaf node ("action" / "condition").
    task_name = spec.get("task")
    if task_name is None:
        raise ValueError(
            f"Mission node {spec.get('id', '')!r} of type {node_type!r} has no "
            f"'task' (expected a composite type {sorted(_COMPOSITES)} or a leaf "
            f"with a 'task' field)."
        )
    try:
        task_cls = registry[task_name]
    except KeyError:
        raise KeyError(
            f"Unknown task '{task_name}'. Known tasks: {sorted(registry)}. "
            f"Is the package that defines it installed and declaring the "
            f"'lotusim.tasks' entry point?"
        )
    return task_cls(
        host=host,
        params=spec.get("params", {}),
        blackboard=blackboard,
        id=spec.get("id", ""),
    )
