{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}

import Control.Monad
import Control.Monad.State

import Data.Maybe
import Data.Map (Map)
import qualified Data.Map as Map
import Control.Lens
import Physics.Bullet
import Control.Concurrent
import Linear

type CubeID = Int
data Cube = Cube
  { _cubBody :: RigidBody
  }
makeLenses ''Cube

data World = World
  { _wldCubes :: !(Map CubeID Cube)
  }
makeLenses ''World

newWorld :: World
newWorld = World mempty

main :: IO ()
main = do
  
  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld 0 0
    
  void . flip runStateT newWorld $ do 
    cubeS <- createBoxShape (V3 1 1 1)
    forM_ [1..1000] $ \i -> do
      rigidBody <- addRigidBody dynamicsWorld i cubeS mempty 
        { rbPosition = V3 0 20 0
        , rbRotation = Quaternion 0.5 (V3 0 1 1)
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = rigidBody
        }
    forever $ do
      
      stepSimulationSimple dynamicsWorld 90

      cubes <- Map.elems <$> use wldCubes
      forM_ cubes $ \cube -> do
        (position, orientation) <- getBodyState (cube ^. cubBody)
        liftIO $ print (position, orientation)
      liftIO $ threadDelay (floor $ 1/60 * 1000000)

