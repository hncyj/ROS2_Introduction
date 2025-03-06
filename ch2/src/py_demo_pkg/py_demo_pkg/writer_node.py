from py_demo_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self, name, age, book:str):
        super().__init__(name, age)
        self.book = book
        print(f"WriterNode constructor called.")
    
    def write(self, tools:str):
        print(f"{self.name} using {tools} write 《{self.book}》.")
        
def main():
    node = WriterNode("chenyinjie", 18, "The Great Gatesby")
    node.write("computer")