import turtle, random


colors = ["red" , "green" , "blue" , "yellow" , "cyan"]
turtle.turtles
turtle.speed('slowest')

radius=50

for _ in range(100):
    angle = random.randint(0,360)
    length =random.randint(20,200)
    circle = random.randint(50, 125)
    turtle.forward(length)
    turtle.right(angle)
    turtle.circle(circle)