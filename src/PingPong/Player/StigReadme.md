# Stig

By Christian Oliveros and Minmin Chen

## Installation

Put `Stig.hs` and the `Stig` folder inside the `src\PingPong\Player` folder.

Also, the next modifications must be done to the Cabal file:

    ...
    exposed-modules:
        PingPong
    other-modules:
        ...
        PingPong.Player.Stig
        PingPong.Player.Stig.General
        PingPong.Player.Stig.GeometryHelpers
        PingPong.Player.Stig.Kinematics
        PingPong.Player.Stig.Fabrik
        ...
    build-depends:
        ...
        , hmatrix
