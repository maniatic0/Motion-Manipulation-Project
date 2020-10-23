module PingPong.Player where

import PingPong.Model
import Data.Geometry
import Control.Lens
import Graphics.Gloss (Color, makeColor, black)
import Data.Char

-- converting game state to and from String

writeState :: Float -> (Float, Item) -> BallState -> Arm -> String
writeState time hit bs a = printTime time ++ printHit hit ++ printBallState bs ++ printArm a

printTime :: Float -> String
printTime t = show t ++ "\n"

printHit :: (Float, Item) -> String
printHit (t, i) = show t ++ " " ++ map toLower (show i) ++ "\n"

printBallState :: BallState -> String
printBallState bs = "loc " ++ printPoint (loc bs) ++ "\n"
                 ++ "dir " ++ printPoint (origin .+^ dir bs) ++ "\n"

printPoint :: Show r => Point 2 r -> String
printPoint p = (show $ view xCoord p) ++ " " ++ (show $ view yCoord p)
  
printArm :: Arm -> String
printArm arm = unwords (map printElement arm)

printElement :: Element -> String
printElement (Joint _ x) = "joint " ++ show x
printElement (Link  _ x) = "link "  ++ show x

readMotion :: String -> Motion
readMotion = map read . words





defaultPlayer :: Player
defaultPlayer = Player
  { name    = "DefaultPlayer"
  , arm     = [ Link black 1, Joint black 0, Link black 0.1 ]
  , initArm = [ Link black 1, Joint black 0, Link black 0.1 ]
  , foot    = 1.5
  , prepare = return ()
  , terminate = return ()
  , action  = const $ const $ const $ const $ return [0]
  , collide = const $ const $ return $ Point2 0 0
  , planPnt = const $ const $ const $ return [0]
  , planSeg = const $ const $ const $ return [0]
  , stretch = defaultStretch
  , dance   = defaultDance
  } 



noCollide :: (Float, Point 2 Float, LineSegment 2 () Float) 
          -> (Float, Point 2 Float, LineSegment 2 () Float) 
          -> IO (Point 2 Float)
noCollide (t1, p1, s1) (t2, p2, s2) = return p2

-- | Default stretching behaviour.
defaultStretch :: Float -> Arm -> IO Motion
defaultStretch t _ = return $ map (* cos t) [-3, 6, -6, 6, -6]

-- | Default dancing behaviour.
defaultDance :: Float -> Arm -> IO Motion
defaultDance t _ = return [ 5 * sin (2.5 * (t + 0.0))
                          , 5 * sin (2.5 * (t + 0.3))
                          , 5 * sin (2.5 * (t + 0.6))
                          , 5 * sin (2.5 * (t + 0.9))
                          , 5 * sin (2.5 * (t + 1.2))
                          ]