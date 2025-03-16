package org.webcam_visual.utils.mat

import org.opencv.core.*

// ---------- Binary operators ----------

// Mat + Mat
operator fun Mat.plus(other: Mat): Mat {
    val result = Mat()
    Core.add(this, other, result)
    return result
}

// Mat + Scalar
operator fun Mat.plus(scalar: Scalar): Mat {
    val result = Mat()
    Core.add(this, scalar, result)
    return result
}

// Mat - Mat
operator fun Mat.minus(other: Mat): Mat {
    val result = Mat()
    Core.subtract(this, other, result)
    return result
}

// Mat - Scalar
operator fun Mat.minus(scalar: Scalar): Mat {
    val result = Mat()
    Core.subtract(this, scalar, result)
    return result
}

// Mat * Mat (element-wise multiplication)
operator fun Mat.times(other: Mat): Mat {
    val result = Mat()
    Core.multiply(this, other, result)
    return result
}

// Mat * Scalar (element-wise multiplication with a scalar)
operator fun Mat.times(scalar: Scalar): Mat {
    val result = Mat()
    Core.multiply(this, scalar, result)
    return result
}

// Mat / Mat (element-wise division)
operator fun Mat.div(other: Mat): Mat {
    val result = Mat()
    Core.divide(this, other, result)
    return result
}

// Mat / Scalar (element-wise division by a scalar)
operator fun Mat.div(scalar: Scalar): Mat {
    val result = Mat()
    Core.divide(this, scalar, result)
    return result
}

// ---------- In-place assignment operators ----------

// In-place addition: A += B
infix fun Mat.plusAssign(other: Mat) {
    Core.add(this, other, this)
}

// In-place addition with a scalar: A += scalar
infix fun Mat.plusAssign(scalar: Scalar) {
    Core.add(this, scalar, this)
}

// In-place subtraction: A -= B
infix fun Mat.minusAssign(other: Mat) {
    Core.subtract(this, other, this)
}

// In-place subtraction with a scalar: A -= scalar
infix fun Mat.minusAssign(scalar: Scalar) {
    Core.subtract(this, scalar, this)
}

// In-place multiplication: A *= B
infix fun Mat.timesAssign(other: Mat) {
    Core.multiply(this, other, this)
}

// In-place multiplication with a scalar: A *= scalar
infix fun Mat.timesAssign(scalar: Scalar) {
    Core.multiply(this, scalar, this)
}

// In-place division: A /= B
infix fun Mat.divAssign(other: Mat) {
    Core.divide(this, other, this)
}

// In-place division with a scalar: A /= scalar
infix fun Mat.divAssign(scalar: Scalar) {
    Core.divide(this, scalar, this)
}

/**
 * Infix function that returns a mask (Mat) where each element is set to 255
 * if the corresponding element in this Mat is greater than the given scalar.
 */
infix fun Mat.gt(scalar: Scalar): Mat {
    val result = Mat()
    Core.compare(this, scalar, result, Core.CMP_GT)
    return result
}

/**
 * Infix function that returns a mask (Mat) where each element is set to 255
 * if the corresponding element in this Mat is less than the given scalar.
 */
infix fun Mat.lt(scalar: Scalar): Mat {
    val result = Mat()
    Core.compare(this, scalar, result, Core.CMP_LT)
    return result
}

/**
 * Infix function that returns a mask (Mat) where each element is set to 255
 * if the corresponding element in this Mat is greater than or equal to the given scalar.
 */
infix fun Mat.ge(scalar: Scalar): Mat {
    val result = Mat()
    Core.compare(this, scalar, result, Core.CMP_GE)
    return result
}

/**
 * Infix function that returns a mask (Mat) where each element is set to 255
 * if the corresponding element in this Mat is less than or equal to the given scalar.
 */
infix fun Mat.le(scalar: Scalar): Mat {
    val result = Mat()
    Core.compare(this, scalar, result, Core.CMP_LE)
    return result
}

/**
 * Infix function that returns a mask (Mat) where each element is set to 255
 * if the corresponding element in this Mat is equal to the given scalar.
 */
infix fun Mat.eq(scalar: Scalar): Mat {
    val result = Mat()
    Core.compare(this, scalar, result, Core.CMP_EQ)
    return result
}

/**
 * Infix function that returns a mask (Mat) where each element is set to 255
 * if the corresponding element in this Mat is not equal to the given scalar.
 */
infix fun Mat.ne(scalar: Scalar): Mat {
    val result = Mat()
    Core.compare(this, scalar, result, Core.CMP_NE)
    return result
}

