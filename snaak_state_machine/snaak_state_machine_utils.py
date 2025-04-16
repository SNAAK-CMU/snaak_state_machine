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
            "timestamp", "success", "duration",
            "bread_desired", "bread_actual",
            "cheese_desired", "cheese_actual",
            "ham_desired", "ham_actual"
        ]
        directory = "/home/snaak/Documents/manipulation_ws/src/snaak_state_machine/results"

        if not os.path.exists(directory):
            os.makedirs(directory)
        self.filepath = os.path.join(directory, "result.csv")
        self.num_placements_attempted = 0
        self.num_placements_succeeded = 0
        self.terminated = False

    def update(self, ingredient_id, num_placed):
        if ingredient_id == "bread_top_slice" or ingredient_id == "bread_bottom_slice":
            ingredient_id = "bread"
        if ingredient_id not in self.desired_dict:
            raise Exception("Invalid Ingredient")
        if ingredient_id != "bread":
            self.num_placements_attempted += 1
            if 0 < num_placed < 3:
                self.num_placements_succeeded += 1 
        # print(f"Placed {num_placed} of {ingredient_id}")
        self.actual_dict[ingredient_id] += num_placed

    def fail(self):
        self.end(fail=True)
        
    def end(self, fail=False):
        if not self.terminated:
            if self.num_placements_attempted == 0 or self.num_placements_succeeded / self.num_placements_attempted < 0.75:
                fail = True

            data = {col: 0 for col in self.COLUMNS}
            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            data.update({
                "timestamp": self.timestamp,
                "success": "Yes" if not fail else "No",
                "duration" : (datetime.now() - datetime.strptime(self.timestamp, "%Y-%m-%d %H:%M:%S")).total_seconds(),
            })
            
            for ingred in self.desired_dict:
                data[f"{ingred}_desired"] = self.desired_dict[ingred]
                data[f"{ingred}_actual"] = self.actual_dict.get(ingred, 0)

            with open(self.filepath, "a", newline="", encoding="utf8") as f:
                writer = csv.DictWriter(f, fieldnames=self.COLUMNS)
                if f.tell() == 0: # if empty, create header
                    writer.writeheader()
                writer.writerow(data)
            self.terminated = True

if __name__ == "__main__":
    ingred= {"cheese" : 2, "ham": 2}
    log = SandwichLogger(ingred)
    log.update("cheese", 2)
    log.update("ham", 1)
    log.update("bread_top_slice", 1)
    log.update("bread_bottom_slice", 1)

    log.end()
    input()
    log = SandwichLogger(ingred)
    log.fail()