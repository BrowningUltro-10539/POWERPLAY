@file:JvmName("MU")

package org.firstinspires.ftc.teamcode.rr.drive


import org.firstinspires.ftc.teamcode.util.units.AngleUnit
import org.firstinspires.ftc.teamcode.util.units.DistanceUnit
import org.firstinspires.ftc.teamcode.util.units.GlobalUnits
import org.firstinspires.ftc.teamcode.util.units.TimeUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue

@JvmOverloads
fun Number.toIn(from: DistanceUnit = GlobalUnits.distance): Double = from.toIn(this)

@JvmOverloads
fun Number.toCm(from: DistanceUnit = GlobalUnits.distance): Double = from.toIn(this) * 2.54

@JvmOverloads
fun Number.toRad(from: AngleUnit = GlobalUnits.angle): Double = from.toDeg(this) * PI / 180

@JvmOverloads
fun Number.toSec(from: TimeUnit = GlobalUnits.time): Double = from.toSec(this)


fun Double.zeroIfNaN() = if (isNaN()) 0.0 else this

fun Float.zeroIfNaN() = if (isNaN()) 0.0f else this


fun Number.isInRange(min: Number, max: Number) = this.toDouble() in min.toDouble()..max.toDouble()

fun Number.clamp(min: Number, max: Number) = this.toDouble().coerceIn(min.toDouble(), max.toDouble())


fun avg(vararg xs: Number) = xs.sumOf { it.toDouble() } / xs.size

fun maxByMagnitude(vararg xs: Number) = xs.maxByOrNull { it.toDouble().absoluteValue } ?: 0.0

fun maxMagnitude(vararg xs: Number) = xs.maxOfOrNull { it.toDouble().absoluteValue } ?: 0.0


@JvmOverloads
fun Number.withDeadzone(deadzone: Number, origin: Number = 0.0): Double {
    val absDelta = abs(this.toDouble() - origin.toDouble())
    val absDeadzone = abs(deadzone.toDouble())

    return if (absDelta < absDeadzone) {
        origin.toDouble()
    } else {
        this.toDouble()
    }
}