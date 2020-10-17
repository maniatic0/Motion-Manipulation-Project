module PingPong.Player.MrStiff (mrStiff) where

import PingPong.Model
import PingPong.Player

import Data.Geometry
import Graphics.Gloss (Color, makeColor)

mrStiff :: Player
mrStiff = defaultPlayer
  { name = "Mr Stiff"
  , arm = stiffArm
  , foot = stiffFoot
  , action = stiffAction
  }

paleBlue :: Color
paleBlue = makeColor 0.5 0.5 0.6 1

hotPink :: Color
hotPink = makeColor 1.0 0.2 0.7 1

stiffArm :: Arm
stiffArm = [ Link paleBlue 0.4
           , Joint paleBlue 0 -- (0.1)
           , Link paleBlue 0.4
           , Joint paleBlue 0 -- (-0.1)
           , Link hotPink 0.1
           ]

stiffFoot :: Float
stiffFoot = 1.3

stiffAction :: Float -> (Float, Item) -> BallState -> Arm -> IO Motion
stiffAction t _ _ _ = return [ 2 * sin (0.27 * t)
                             , 2 * sin (1.03 * t)
                             ]

