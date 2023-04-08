# ROS2 Guessing game

This is a demo app to play around with ROS2, it's a Guessing game where the server selects a random number and the client will try to guess it.

This project creates two nodes, one **dealer** and one **player**, both contained in the `guessing_game` package, that communicate with each other in two ways, a _Topic_ and an _Action_.

In the `guessing_interfaces` package the action _Guessing_ and the message type _Guess_ are defined

## Requirements

This project is built around the Humble Hawksbill distribution of ROS2, please see the official installation instructions [here](https://docs.ros.org/en/humble/Installation.html)

## Installation

* Copy this repo
* Build the project with `colcon`

```bash
colcon build
```

## Usage

Load the environment variables into the system
Run the server node

```bash
source install/setup.bash
ros2 run guessing_game guessing_dealer
```

In a different terminal, run the player

```bash
source install/setup.bash
ros2 run guessing_game guessing_player
```

## TODO

* Allow for multiple players
* Start a new round after the winning number is guessed

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

## License

[MIT](https://choosealicense.com/licenses/mit/)
