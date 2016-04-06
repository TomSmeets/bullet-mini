{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.CollisionShape where

import qualified Language.C.Inline.Cpp as C

import Data.Monoid
import Data.Vector.Storable.Mutable (IOVector)
import Foreign.C
import Foreign.Ptr
import Linear.Extra

C.context (C.cppCtx <> C.funCtx <> C.vecCtx)

C.include "<btBulletDynamicsCommon.h>"

newtype CollisionShape = CollisionShape (Ptr ())
newtype MeshInterface = MeshInterface (Ptr ())

boxShape :: V3 CFloat -- ^ half box size
         -> IO CollisionShape
boxShape (V3 sx sy sz) = CollisionShape <$> [C.block| void* {
    btVector3 s = btVector3($(float sx), $(float sy), $(float sz));
    btCollisionShape* shape = new btBoxShape(s);
    return shape;
    }|]

sphereShape :: CFloat -- ^ sphere radius
            -> IO CollisionShape
sphereShape radius = CollisionShape <$> [C.block| void* {
    btCollisionShape* shape = new btSphereShape($(float radius));
    return shape;
    }|]

cylinderShape :: V3 CFloat -- ^ half size
              -> IO CollisionShape
cylinderShape (V3 sx sy sz) = CollisionShape <$> [C.block| void* {
    btVector3 n = btVector3($(float sx), $(float sy), $(float sz));
    btCollisionShape* shape = new btCylinderShape(n);
    return shape;
    }|]

capsuleShape :: CFloat -- ^ radius
             -> CFloat -- ^ height
             -> IO CollisionShape
capsuleShape radius height = CollisionShape <$> [C.block| void* {
    btCollisionShape* shape = new btCapsuleShape($(float radius), $(float height));
    return shape;
    }|]

coneShape :: CFloat -- ^ radius
          -> CFloat -- ^ height
          -> IO CollisionShape
coneShape radius height = CollisionShape <$> [C.block| void* {
    btCollisionShape* shape = new btConeShape($(float radius), $(float height));
    return shape;
    }|]

staticPlaneShape :: V3 CFloat -- ^ plane normal
                 -> CFloat    -- ^ half plane offset along normal
                 -> IO CollisionShape
staticPlaneShape (V3 nx ny nz) offset = CollisionShape <$> [C.block| void* {
    btVector3 n = btVector3($(float nx), $(float ny), $(float nz));
    btCollisionShape* shape = new btStaticPlaneShape(n, $(float offset));
    return shape;
    }|]

convexHullShape :: CInt -> IOVector CFloat -> IO CollisionShape
convexHullShape numPoints points = CollisionShape <$> [C.block| void* {
    btCollisionShape* shape = new btConvexHullShape($vec-ptr:(float* points), $(int numPoints));
    return shape;
    }|]

triangleMesh :: CInt -> IOVector CInt -> CInt -> IOVector CFloat -> IO MeshInterface
triangleMesh numTriangles triangles numPoints points = MeshInterface <$> [C.block| void* {
        // Haskell garbage-collecs these vectors.
        // Make a copy!
        int* triangles = new int[$vec-len:triangles];
        float* points = new float[$vec-len:points];
        memcpy(triangles, $vec-ptr:(int* triangles), $vec-len:triangles * sizeof(int));
        memcpy(points,    $vec-ptr:(float* points),  $vec-len:points    * sizeof(float));

        btStridingMeshInterface* mesh = new btTriangleIndexVertexArray($(int numTriangles), triangles, 3*sizeof(int), $(int numPoints), points, 3*sizeof(btScalar));
        return mesh;
    }|]

convexTriangleMeshShape :: MeshInterface -> IO CollisionShape
convexTriangleMeshShape (MeshInterface mesh) = CollisionShape <$> [C.block| void* {
    btCollisionShape* shape = new btConvexTriangleMeshShape((btStridingMeshInterface*)$(void* mesh));
    return shape;
    }|]

bvhTriangleMeshShape :: MeshInterface -> IO CollisionShape
bvhTriangleMeshShape (MeshInterface mesh) = CollisionShape <$> [C.block| void* {
    btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape((btStridingMeshInterface*)$(void* mesh),true,true);
    return (btCollisionShape*) shape;
    }|]

-- ToDo:
-- * multiSphereShape
