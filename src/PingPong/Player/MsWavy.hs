module PingPong.Player.MsWavy (msWavy) where

import Control.Lens
import Data.Geometry

import PingPong.Model
import PingPong.Player

import Graphics.Gloss (Color, makeColor)

msWavy :: Player
msWavy = Player wavyArm wavyFoot wavyAction noCollide

gradient :: Float -> Color
gradient x = makeColor x 0.8 0.2 1



wavyArm :: Arm
wavyArm = [ Link (gradient 0.3) 0.2
          , Joint (gradient 0.35) 0.0
          , Link (gradient 0.4) 0.2
          , Joint (gradient 0.45) 0.0
          , Link (gradient 0.5) 0.2
          , Joint (gradient 0.55) 0.0
          , Link (gradient 0.6) 0.2
          , Joint (gradient 0.65) 0.0
          , Link (gradient 0.7) 0.1
          ]

wavyFoot :: Float
wavyFoot = 1.5

wavyAction :: BallState -> Arm -> IO Motion
wavyAction bs _ = return $ let t = view xCoord $ loc bs
                  in [ -0.5 * sin (2.2 * t)
                     , -0.5 * cos (2.3 * t)
                     ,  0.5 * sin (2.4 * t)
                     ,  0.5 * cos (2.5 * t)
                     ]


noCollide :: (Float, Point 2 Float, LineSegment 2 () Float) 
          -> (Float, Point 2 Float, LineSegment 2 () Float) 
          -> Point 2 Float
noCollide (t1, p1, s1) (t2, p2, s2) = p2                     