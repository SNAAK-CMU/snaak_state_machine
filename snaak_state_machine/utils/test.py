from __future__ import annotations
import os
import yaml
from pathlib import Path
from typing import Dict,Any
from dataclasses import dataclass
from typing import Dict, List, Tuple, Iterable, Optional
from collections import Counter
import yaml
from copy import deepcopy



stock_file_path = "/home/snaak/Documents/recipe/stock.yaml"

recipe = {'white_bread':{'slices_req':2},
          'peperoni': {'slices_req':2}}

stock = {'white_bread':{'slices':6,'bin':1,'type':'bread','weight_per_slice':20},
            'peperoni': {'slices':4,'bin':2,'type':'meat','weight_per_slice':10},
            'cheddar': {'slices':4,'bin':3,'type':'cheese','weight_per_slice':15}
        }         

# print(list(recipe.keys())[0])
# recipe_ingredients = list(recipe.keys())

# print(stock[recipe_ingredients[0]]['slices'])

RECIPE_YAML_PATH = "/home/snaak/Documents/recipe/recipe.yaml"  # adjust if needed

stock_file_path = "/home/snaak/Documents/recipe/stock.yaml"

def load_recipe_dict(yaml_path: str) -> Dict[str, Dict[str, int]]:
    """
    Parse YAML of form:
    recipe:
      - white_bread: 2
      - peperoni: 2
      - any_other: N

    Returns: {'white_bread': {'slices_req': 2}, ...}
    """
    p = Path(yaml_path)
    if not p.is_file():
        raise FileNotFoundError(f"File not found: {yaml_path}")

    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    raw = data.get("recipe")
    if raw is None:
        raise ValueError("YAML missing top-level 'recipe' key")

    recipe: Dict[str, Dict[str, int]] = {}

    if isinstance(raw, list):
        for item in raw:
            if not isinstance(item, dict):
                raise ValueError(f"Invalid list entry (expected mapping): {item}")
            for name, qty in item.items():
                recipe[name] = {"slices_req": int(qty)}
    elif isinstance(raw, dict):
        # (Also support mapping form)
        for name, qty in raw.items():
            recipe[name] = {"slices_req": int(qty)}
    else:
        raise ValueError("'recipe' must be a list or a mapping")

    return recipe

def load_stock_dict(yaml_path: str) -> Dict[str, Dict[str, Any]]:
    """
    Parse YAML of form:

    ingredients:
      cheddar:
        slices: 4
        bin: 1
        type: cheese
        weight_per_slice: 20.5
      ham:
        slices: 4
        bin: 2
        type: meat
        weight_per_slice: 37
      italian_white_bread:
        slices: 4
        bin: 3
        type: bread
        weight_per_slice: 32

    Returns:
      {
        'cheddar': {'slices': 4, 'bin': 1, 'type': 'cheese', 'weight_per_slice': 20.5},
        'ham': {...},
        'italian_white_bread': {...}
      }
    """
    p = Path(yaml_path)
    if not p.is_file():
        raise FileNotFoundError(f"File not found: {yaml_path}")

    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    ing = data.get("ingredients")
    if ing is None or not isinstance(ing, dict):
        raise ValueError("YAML missing 'ingredients' mapping")

    stock: Dict[str, Dict[str, Any]] = {}
    for name, info in ing.items():
        if not isinstance(info, dict):
            raise ValueError(f"Entry for '{name}' must be a mapping")
        try:
            slices = int(info.get("slices", 0))
            bin_id = int(info.get("bin", 0))
            type_ = str(info.get("type", ""))
            wps = float(info.get("weight_per_slice", 0.0))
        except (TypeError, ValueError) as e:
            raise ValueError(f"Bad field types for ingredient '{name}': {e}")
        stock[name] = {
            "slices": slices,
            "bin": bin_id,
            "type": type_,
            "weight_per_slice": wps,
        }
    return stock


# ---------- dataclasses ----------


if __name__ == "__main__":
    all_breads = [k for k, v in stock.items() if v.get('type') == 'bread' and k in list(recipe.keys())]
    # print(recipe[all_breads[0]]['slices_req'])
    print(all_breads)