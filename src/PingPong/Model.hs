module PingPong.Model where

import Transformation

import Data.Geometry hiding (init)
import Data.Geometry.Matrix
import Data.Geometry.Transformation
import Data.Geometry.PolyLine
import Data.Geometry.Polygon

import Data.Ext
import Control.Lens

import Graphics.Gloss (Color)



-- * Relevant Assignment Data

-- | Specifies a component which makes up the robotic arm of the player.
-- An arm always consist of revolute Joints which are connected with Links.
data Element = Link Color Float
             | Joint Color Float
  deriving (Show, Eq)

-- | Checks if the provided element is a revolute joint.
isJoint :: Element -> Bool
isJoint (Joint _ _) = True
isJoint _           = False

-- | A robotic arm is a combination of multiple components or elements.
type Arm = [Element]

-- | The current time as specified within the simulation.
type GameTime = Float
-- | The last moment something hit the ball.
-- This contains both the time of the hit and what got hit.
type Hit = (GameTime, Item)

-- | The ping pong ball is defined as residing at a position in space
-- and having a certain velocity defined as its direction.
data BallState = BallState {
  -- | Provides the position where the ball currently resides.
  loc :: Point 2 Float,
  -- | Provides the direction and velocity the ball is moving in.
  dir :: Vector 2 Float
} deriving (Show, Eq)

-- | Specifies a single player or character that can play table tennis.
data Player = Player {
  -- | Specifies a humanly readable name to call this beautiful arm.
  name   :: String,
  -- | Specifies the composition of the robotic arm which can move around.
  arm    :: Arm,
  -- | Specifies how far from the origin the arm starts.
  foot   :: Float,
  -- | Setups external servers or initial data so that this player can work
  -- flawlessly during executing. This method is only called once at the start
  -- of the game, NOT every single round.
  prepare :: IO (),
  -- | Cleans up all executing and remaining data from the current player.
  -- This will only be called once at the end of the simulation.
  terminate :: IO (),
  -- | Provides the action that the robotic arm should perform given
  -- the current state of the world and its own position in space.
  action :: GameTime -> Hit -> BallState -> Arm -> IO Motion,
  -- | Makes the arm stretch before the start of the game.
  stretch :: GameTime -> Arm -> IO Motion,
  -- | Makes the robotic arm do a victory dance when it has scored a point.
  dance :: GameTime -> Arm -> IO Motion,
  -- | Calculate the location of a ball after a collision with the racket.
  --
  -- This method requires 2 states in order to calculate the collision,
  -- the frame before the collision took place and the frame after the collision
  -- happened assuming there was no intersection. This method will find the
  -- frame inbetween and recalculate the position of where the ball was supposed
  -- to be.
  --
  -- Each state is represented as a tuple of the time, ball location and
  -- the racket location.
  collide :: (Float, Point 2 Float, LineSegment 2 () Float)
          -> (Float, Point 2 Float, LineSegment 2 () Float)
          -> IO (Point 2 Float),
  -- | Calculates a path for the provided robotic arm to move its
  -- racket on top of the specified point. The path is encoded as the angles
  -- that the current robotic arm should move to reach that specific point.
  --
  -- This function does not account for internal collisions of the arm,
  -- and allows for infinite revolutions on all joints.
  planPnt :: Float          -- ^ The x-coordinate of the arm's base.
          -> Arm            -- ^ The setup of the robotic arm at the start.
          -> Point 2 Float  -- ^ The point in space which should be reached.
          -> IO Motion,     -- ^ The motions for each joint to reach the end.
  -- | Calculates a path for the provided robotic arm to move its
  -- racket to the provided position. The path is encoded as the angles
  -- that the current robotic arm should move to reach that specific point.
  -- The racket can be on this line in 2 configurations: straight-up and
  -- upside-down.
  --
  -- This function does not account for internal collisions of the arm,
  -- and allows for infinite revolutions on all joints.
  planSeg :: Float          -- ^ The x-coordinate of the arm's base.
          -> Arm            -- ^ The setup of the robotic arm at the start.
          -> LineSegment 2 () Float -- ^ The goal position of the racket.
          -> IO Motion,     -- ^ The motions for each joint to reach the end.
  -- | Created the arm itself at the beginning of a match.
  initArm :: Arm
}

-- | The motion that each individual joint should make, stored in an array.
-- The speed is defined in radians per second at which joints can move.
type Motion = [Float]

-- * Simulation Data

-- | Data type describing the last thing hit by the ball
-- At the beginning of the game this will be Opponent's bat to represent
-- them serving, even if in the simulations this never happened.
data Item =
  -- | Within a single frame, the value Air is used to correctly handle multiple
  -- collisions.
  -- This value will never be recorded.
  Air
  -- | Specifies that the ball hit a player's bat.
  | Bat Owner
  -- | Specifies that the ball hit the table last on the provided side.
  | Table Owner
  | Net
  -- | Specifies that the ball hit any other surface denoted by an identifier.
  -- This will be the walls of the room, the ceiling or the floor, the underside
  -- of the table, etc. depending on the location of the ball.
  | Other Int
  deriving (Show, Eq, Ord)

-- | Specifies which player was responsible for a certain action.
-- This is used to describe whose side of the table or whose bat got hit last.
data Owner =
  -- | The individual responsible for a certain action was the current player.
  Self
  -- | The individual responsible for a certain action was the opposing player.
  | Opponent
  deriving (Show, Eq, Ord)

-- | Describes the current situation of the field to determine arm behaviour.
data Phase =
    -- | The situation when setting up and initialising the robotic arms.
    BeforeGame Float
    -- | The situation before the players will face off against each other.
    -- This is used to attempt to revert the players back to 0.
    | BeforeRally Float
    -- | The situation where two players are facing off against each other.
    -- This is the main match when they are playing.
    | DuringRally
    -- | The situation after a set has been played and someone scored a point.
    -- This allows for victory dances of each robotic arm.
    | AfterRally Float
    -- | The situation after all sets have been played and a definitive winner
    -- has been chosen! Now the robotic arms can be cleaned up and terminate
    -- removing all remaining data from both players.
    | AfterGame Float
    -- | State to indicate the process should be terminated.
    | GameOver    
  deriving (Eq, Show)

data State = State {
    -- | The current phase or progress within a game.
    phase  :: Phase,
    -- | The time elapsed since the start of the game.
    time   :: GameTime,
    -- | The number of frames since the start of the game.
    frame  ::  Int,
    -- | The number of points scored by each player.
    score  :: (Int, Int),
    -- | The location of the ball in space currently.
    ball   ::  BallState,
    -- | The time and location of the last collision.
    hits   :: [Hit],
    -- | One of the players who participate in this game.
    p1     ::  Player,
    -- | One of the players who participate in this game.
    p2     ::  Player,
    -- | One of the motions until the next frame.
    m1     ::  Motion,
    -- | One of the motions until the next frame.
    m2     ::  Motion
  }


-- | Geometry of the room.
room :: SimplePolygon () Float
room = Data.Geometry.Polygon.fromPoints $ map (:+ ()) [Point2 (-3) 0, Point2 3 0, Point2 3 6, Point2 (-3) 6]

-- | Geometry of the table.
table :: SimplePolygon () Float
table = Data.Geometry.Polygon.fromPoints $ map (:+ ()) [Point2 1 0.5, Point2 0 0.5, Point2 (-1) 0.5, Point2 (-1) 0.4, Point2 1 0.4]

-- | Geometry of the net.
net :: LineSegment 2 () Float
net = ClosedLineSegment (Point2 0 0.5 :+ ()) (Point2 0 0.6 :+ ())





transformation :: Element -> Transformation 2 Float
transformation (Link _ d) = translation $ Vector2 0 d
transformation (Joint _ a) = rotation a

transformations :: Arm -> [Transformation 2 Float]
transformations = map transformation

globalize :: [Transformation 2 Float] -> [Transformation 2 Float]
globalize ts = scanl (|.|) identity ts

evaluate :: Arm -> [Point 2 Float]
evaluate arm = map (origin .+^) 
             $ map (flip transformBy $ Vector2 0 0) 
             $ globalize $ transformations arm

evaluateP :: Arm -> PolyLine 2 () Float
evaluateP arm = fromPointsUnsafe
              $ map (:+ ())
              $ evaluate arm




armSegments :: Player -> Bool -> [LineSegment 2 () Float]
armSegments p f = 
  let rps = evaluate $ arm p
      trl = translation (Vector2 (foot p) 0)
      scl = scaling (Vector2 (if f then 1 else -1) 1)
      nps = map (transformBy (scl |.| trl)) rps
      fps = map (:+ ()) nps
  in zipWith (OpenLineSegment) (init fps) (tail fps)




 







-- | Change state from the perspective of p1 to the perspective of p2.
flipState :: State -> State
flipState st = st { ball = flipBall $ ball st
                  , hits = map flipHit $ hits st
                  , p1 = p2 st
                  , p2 = p1 st
                  , m1 = flipMotion $ m2 st
                  , m2 = flipMotion $ m1 st
                  }

flipBall :: BallState -> BallState
flipBall st = st { loc = transformBy reflectionH $ loc st
                 , dir = transformBy reflectionH $ dir st
                 }

flipMotion :: Motion -> Motion
flipMotion = map negate

flipHit :: (Float, Item) -> (Float, Item)
flipHit (f, i) = (f, flipItem i)

flipItem :: Item -> Item
flipItem (Bat   o) = Bat   $ flipOwner o
flipItem (Table o) = Table $ flipOwner o
flipItem o         = o

flipOwner :: Owner -> Owner
flipOwner Self = Opponent
flipOwner Opponent = Self

defState :: State
defState = State
  { phase = BeforeGame 2
  , time  = 0
  , frame = 0
  , score = (0, 0)
  , ball  = BallState (Point2 0 1) (Vector2 1 0)
  , hits  = [(0, Bat Opponent)]
  , p1    = undefined
  , p2    = undefined
  , m1    = undefined
  , m2    = undefined
  }

winner :: State -> Player
winner s | fst (score s) > snd (score s) = p1 s
         | otherwise = p2 s

