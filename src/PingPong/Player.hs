module PingPong.Player where

import PingPong.Model
import Data.Geometry
import Control.Lens


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





