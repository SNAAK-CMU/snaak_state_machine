from datetime import datetime
import csv
import os

class SandwichLogger():
    def __init__(self, ingredients):
        self.desired_dict = ingredients
        self.desired_dict["bread"] = 2
        self.actual_dict = {ingredient: 0 for ingredient in self.desired_dict}
        self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")    
        self.COLUMNS = [
            "timestamp", "success",
            "bread_desired", "bread_actual",
            "cheese_desired", "cheese_actual",
            "ham_desired", "ham_actual"
        ]
        directory = "/home/oberton/snaak_ws/src/snaak_state_machine/log"
        if not os.path.exists(directory):
            os.makedirs(directory)
        self.filepath = os.path.join(directory, "result.csv")
        self.num_placements_attempted = 0
        self.num_placements_succeeded = 0

    def update(self, ingredient_id, num_placed):
        if ingredient_id not in self.desired_dict:
            raise Exception("Invalid Ingredient")
        if ingredient_id != "bread":
            self.num_placements_attempted += 1
            if 0 < num_placed < 3:
                self.num_placements_succeeded += 1
        self.actual_dict[ingredient_id] += num_placed

    def fail(self):
        self.end(fail=True)
        
    def end(self, fail=False):
        if self.num_placements_attempted == 0 or self.num_placements_succeeded / self.num_placements_attempted < 0.8:
            fail = True

        data = {col: 0 for col in self.COLUMNS}
        data.update({
            "timestamp": self.timestamp,
            "success": "Yes" if not fail else "No"
        })
        
        for ingred in self.desired_dict:
            data[f"{ingred}_desired"] = self.desired_dict[ingred]
            data[f"{ingred}_actual"] = self.actual_dict.get(ingred, 0)

        with open(self.filepath, "a", newline="", encoding="utf8") as f:
            writer = csv.DictWriter(f, fieldnames=self.COLUMNS)
            if f.tell() == 0: # if empty, create header
                writer.writeheader()
            writer.writerow(data)

if __name__ == "__main__":
    ingred= {"cheese" : 2, "bologna": 2}
    log = SandwichLogging(ingred)
    log.update("cheese", 2)
    log.update("bologna", 1)
    log.update("bread", 2)
    log.end()
    input()
    log = SandwichLogging(ingred)
    log.fail()