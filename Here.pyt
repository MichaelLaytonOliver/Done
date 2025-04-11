import turtle, random


colors = ["red" , "green" , "blue" , "yellow" , "cyan"]
turtle.turtles
turtle.speed('slowest')
radius=50

turtle.onclick(turtle.circle(30))



while True:
    angle = random.randint(0,360)
    length =random.randint(20,200)
    circle = random.randint(50, 125)
    turtle.forward(length)
    turtle.right(angle)
    turtle.circle(circle)
    angle = random.randint(0,360)
    turtle.left(angle)