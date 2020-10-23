module PingPong.Player.MsWavy (msWavy) where

import Control.Lens
import Data.Geometry

import PingPong.Model
import PingPong.Player

import Graphics.Gloss (Color, makeColor)

msWavy :: Player
msWavy = defaultPlayer
  { name = "Ms Wavy"
  , arm = wavyArm
  , foot = wavyFoot
  , action = wavyAction
  , collide = noCollide
  }

gradient :: Float -> Color
gradient x = makeColor x 0.8 0.2 1



wavyArm :: Arm
wavyArm = [ Link (gradient 0.3) 0.2
          , Joint (gradient 0.35) (-0.3)
          , Link (gradient 0.4) 0.2
          , Joint (gradient 0.45) (0.2)
          , Link (gradient 0.5) 0.2
          , Joint (gradient 0.55) (0.2)
          , Link (gradient 0.6) 0.2
          , Joint (gradient 0.65) (-0.1)
          , Link (gradient 0.7) 0.1
          ]

wavyFoot :: Float
wavyFoot = 1.5

wavyAction :: Float -> (Float, Item) -> BallState -> Arm -> IO Motion
wavyAction t _ _ _ = return [ -2 * sin (2.2 * t)
                            , -2 * cos (2.3 * t)
                            ,  2 * sin (2.4 * t)
                            ,  2 * cos (2.5 * t)
                            ]

