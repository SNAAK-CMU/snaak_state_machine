from datetime import datetime
import csv
from ament_index_python.packages import get_package_share_directory
import os

class SandwichLogging():
    def __init__(self, ingredients):
        self.desired_dict = ingredients
        self.desired_dict["bread"] = 2
        self.actual_dict = {}
        self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")    
        self.COLUMNS = [
            "timestamp", "success",
            "bread_desired", "bread_actual",
            "cheese_desired", "cheese_actual",
            "bologna_desired", "bologna_actual"
        ]
        self.filepath = "/home/snaak/Documents/manipulation_ws/src/snaak_state_machine/log/results.csv",


    def update(self, ingredient_id, num_placed):
        self.actual_dict[ingredient_id] += num_placed
    
    def fail(self):
        self.end(fail=True)
        
    def end(self, fail=False):
        num_fails = 0
        num_ingreds = 0
        ingred_data = {}
        for ingred in self.desired_dict:
            desired = self.desired_dict[ingred]
            actual = self.actual_dict.get(ingred, 0)
            ingred_data[f"{ingred}_desired"] = desired
            ingred_data[f"{ingred}_actual"] = actual
            
            if ingred != "bread":
                num_ingreds += desired
                num_fails += desired - actual

        if num_ingreds > 0 and num_fails / num_ingreds > 0.2:
            fail = True

        data = {col: 0 for col in self.COLUMNS}
        data.update({
            "timestamp": self.timestamp,
            "success": not fail
        })
        
        for ingred in self.desired_dict:
            data[f"{ingred}_desired"] = self.desired_dict[ingred]
            data[f"{ingred}_actual"] = self.actual_dict.get(ingred, 0)

        with open(self.filepath, "a", newline="", encoding="utf8") as f:
            writer = csv.DictWriter(f, fieldnames=self.FIXED_COLUMNS)
            if f.tell() == 0: # if empty, create header
                writer.writeheader()
            writer.writerow(data)




