"""Guard Python 3.10 compatibility.

Real mode imports the simulation adapters into the middleware process, and ROS 2
Humble's rclpy only runs on Python 3.10 (Ubuntu 22.04). Names added to ``typing`` in
3.11+ must come from ``typing_extensions`` instead. This is a static check because the
development machines do not have Python 3.10.
"""

import ast
from pathlib import Path

import pytest

MIDDLEWARE = Path(__file__).resolve().parents[1]
SOURCES = sorted(
    path
    for folder in ("app", "scripts")
    for path in (MIDDLEWARE / folder).rglob("*.py")
)
# typing names that need Python 3.11+ (TypedDict: pydantic requires typing_extensions < 3.12)
TYPING_311 = {
    "Self", "NotRequired", "Required", "LiteralString", "Never", "assert_never",
    "reveal_type", "Unpack", "TypeVarTuple", "dataclass_transform", "TypedDict",
}
STDLIB_311 = {"tomllib", "wsgiref.types"}


@pytest.mark.parametrize("path", SOURCES, ids=lambda p: str(p.relative_to(MIDDLEWARE)))
def test_source_avoids_python_311_only_imports(path: Path) -> None:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    problems = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "typing":
            problems += [alias.name for alias in node.names if alias.name in TYPING_311]
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            modules = [node.module] if isinstance(node, ast.ImportFrom) else [
                alias.name for alias in node.names
            ]
            problems += [m for m in modules if m in STDLIB_311]
        if isinstance(node, ast.Try) and any(
            isinstance(handler, ast.ExceptHandler) and getattr(handler, "type", None)
            and isinstance(handler.type, ast.Name) and handler.type.id == "ExceptionGroup"
            for handler in node.handlers
        ):
            problems.append("ExceptionGroup")
    assert not problems, f"use typing_extensions / avoid 3.11+ features: {problems}"
