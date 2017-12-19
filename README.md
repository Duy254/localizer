# localizer

This node acts as the main package for all the localization stuff of Cabot.

## Code Structure

The main file, `src/localizer_node.cpp` is designed to contain as least code as possible, so that the reader can see clearly what is going on.

The class `Fusion` contains all the heavy lifting including `subscribe` and `publish`

Currently this only get information from Navcog.

## What's subscribed and published

- Subscribed
    - `/NavCog/pose`: The location info from NavCog
- Published
    - `/odometry`: a `navcog_msg/SimplifiedOdometry` message, containing fused pose info
