{-# LANGUAGE TypeOperators #-}
module Main where

import Control.Concurrent
import Linear.Extra
import Physics.Bullet

dt :: Float
dt = 1 / 60

main :: IO ()
main = do
    putStrLn "Test"
    dynamicsWorld  <- createDynamicsWorld DynamicsWorldConfig{ dwGravity = -9.81 }

    -- ground plane
    groundShape <- staticPlaneShape (V3 0 1 0) 0
    _ <- addRigidBody dynamicsWorld groundShape mempty{ rbMass = 0 }

    ballShape <- sphereShape 1
    ball <- addRigidBody dynamicsWorld ballShape mempty{ rbPosition = V3 0 10 0
                                                       , rbMass = 1
                                                       }

    let loop = do stepSimulationWithTimestep dynamicsWorld (realToFrac dt) 10 dt

                  bst <- getBodyState ball :: IO (V3 Float, Quaternion Float)
                  print bst

                  threadDelay (floor $ dt * 1000000)
                  loop

    loop
