{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.CollisionObject where

import qualified Language.C.Inline.Cpp as C

import Control.Monad.Trans
import Data.Binary
import Data.Monoid
import Foreign.C
import Foreign.Ptr
import Linear.Extra
import Physics.Bullet.CollisionShape
import Physics.Bullet.DynamicsWorld
import Physics.Bullet.Types

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"

newtype CollisionObjectID = CollisionObjectID { unCollisionObjectID :: Word32 }
    deriving (Eq, Show, Ord, Binary, Num, Enum, Real, Integral)

newtype CollisionObject = CollisionObject { unCollisionObject :: Ptr () } deriving Show

instance ToCollisionObjectPointer CollisionObject where
  toCollisionObjectPointer = unCollisionObject

addCollisionObject :: DynamicsWorld -> CollisionShape -> IO CollisionObject
addCollisionObject (DynamicsWorld world) (CollisionShape shape) = CollisionObject <$> [C.block| void* {
        btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*) $(void* world);
        btCollisionShape* shape = (btCollisionShape*) $(void* shape);
        btCollisionObject* obj = new btCollisionObject();
        obj->setCollisionShape(shape);
        world->addCollisionObject(obj);
        return obj;
    }|]

getCollisionObjectID :: (ToCollisionObjectPointer a, MonadIO m) => a -> m CollisionObjectID
getCollisionObjectID (toCollisionObjectPointer -> collisionObject) = liftIO $
  fromIntegral <$> [C.block| int {
    btCollisionObject* collisionObject = (btCollisionObject*)$(void* collisionObject);
    int collisionObjectID = collisionObject->getUserIndex();
    return collisionObjectID;
    }|]

-- | Don't use this for rigid bodies! Use setRigidBodyWorldTransform instead, as that updates the MotionState
setCollisionObjectWorldTransform :: (ToCollisionObjectPointer a, Real b, MonadIO m) => a -> V3 b -> Quaternion b -> m ()
setCollisionObjectWorldTransform (toCollisionObjectPointer -> collisionObject) position rotation = liftIO [C.block| void {
  btCollisionObject* collisionObject     = (btCollisionObject*) $(void* collisionObject);

  btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
  btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );
  collisionObject->setWorldTransform( btTransform(q , p) );
  }|]
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation
