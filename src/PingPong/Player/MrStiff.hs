module PingPong.Player.MrStiff (mrStiff) where

import PingPong.Model
import PingPong.Player

import Data.Geometry
import Graphics.Gloss (Color, makeColor)

mrStiff :: Player
mrStiff = Player stiffArm stiffFoot noAction noCollide

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



noAction :: BallState -> Arm -> IO Motion
noAction _ _ = return [0, 0]

noCollide :: (Float, Point 2 Float, LineSegment 2 () Float) 
          -> (Float, Point 2 Float, LineSegment 2 () Float) 
          -> Point 2 Float
noCollide (t1, p1, s1) (t2, p2, s2) = p2