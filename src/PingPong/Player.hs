module PingPong.Player where

import PingPong.Model
import Data.Geometry
import Control.Lens
import Graphics.Gloss (Color, makeColor, black)

-- converting game state to and from String

writeState :: BallState -> Arm -> String
writeState bs a = printBallState bs ++ printArm a

printBallState :: BallState -> String
printBallState bs = "loc " ++ printPoint (loc bs) ++ "\n"
                 ++ "dir " ++ printPoint (origin .+^ dir bs) ++ "\n"

printPoint :: Show r => Point 2 r -> String
printPoint p = (show $ view xCoord p) ++ " " ++ (show $ view yCoord p)
  
printArm :: Arm -> String
printArm arm = "arm " ++ unwords (map printElement arm)

printElement :: Element -> String
printElement (Joint _ x) = "joint " ++ show x
printElement (Link  _ x) = "link  " ++ show x

readMotion :: String -> Motion
readMotion = map read . words





defaultPlayer :: Player
defaultPlayer = Player
  { name    = "DefaultPlayer"
  , arm     = [ Link black 1, Joint black 0, Link black 0.1 ]
  , foot    = 1.5
  , action  = const $ const $ return [0]
  , collide = const $ const $ Point2 0 0
  , planPnt = const $ const $ const $ [0]
  , planSeg = const $ const $ const $ [0]
  } 




noAction :: BallState -> Arm -> IO Motion
noAction _ _ = return [0, 0]

noCollide :: (Float, Point 2 Float, LineSegment 2 () Float) 
          -> (Float, Point 2 Float, LineSegment 2 () Float) 
          -> Point 2 Float
noCollide (t1, p1, s1) (t2, p2, s2) = p2