
# Mathematical Fundamentals of a 2D Lookup Table with Bilinear Interpolation

## 1. Introduction

A 2D lookup table is a data structure used to approximate a function of two variables, `z = f(x, y)`. Instead of calculating the function directly, which might be computationally expensive or based on empirical data, we pre-calculate or measure its value at specific points on a grid and store them.

When a value is needed for a point `(x, y)` that is not exactly on a grid point, we need a mathematical method to estimate it based on the surrounding known points. The `lookup_table_2d` implementation uses **bilinear interpolation** for this estimation and **clamping** to handle queries outside the grid's boundaries.

This document details the mathematics behind these two core concepts.

## 2. The Data Grid

The foundation of the lookup table is a discrete grid defined by three components:

1.  **X-Axis Breakpoints:** A sorted array of `NumX` coordinates along the x-axis.
    `x_axis = {x₀, x₁, ..., x_{NumX-1}}`
2.  **Y-Axis Breakpoints:** A sorted array of `NumY` coordinates along the y-axis.
    `y_axis = {y₀, y₁, ..., y_{NumY-1}}`
3.  **Z-Values:** A 2D array of `NumY × NumX` data points, where `z[j][i]` corresponds to the function value at the grid point `(xᵢ, yⱼ)`.

Visually, this forms a grid of rectangular cells:

```
      y
      ▲
      │
y₂ ---+-------Q₀₂------Q₁₂------Q₂₂-----+
      │         |        |        |     |
      │         |        |        |     |
y₁ ---+-------Q₀₁------Q₁₁------Q₂₁-----+
      │         |        |        |     |
      │         |        |        |     |
y₀ ---+-------Q₀₀------Q₁₀------Q₂₀-----+
      │
      +------------------------------------► x
              x₀       x₁       x₂
```
Here, `Qᵢⱼ` represents the data point `z[j][i]` at coordinate `(xᵢ, yⱼ)`.

## 3. The Algorithm: `get_value(x, y)`

The process of finding the value `z` for an arbitrary input `(x, y)` involves three main steps.

### Step 1: Locate the Interpolation Cell

Given an input point `P = (x, y)`, the first step is to identify the rectangular cell that contains it. This means finding the indices `i` and `j` such that:

*   `xᵢ ≤ x < xᵢ₊₁`
*   `yⱼ ≤ y < yⱼ₊₁`

This is accomplished by performing a search (e.g., `std::lower_bound`) on the `x_axis` and `y_axis` arrays to find the indices of the "lower-left" corner of the cell, which we'll call `(xᵢ, yⱼ)`.

The four corners of this cell are:
*   `Q₀₀ = (xᵢ,   yⱼ)`   with value `z[j][i]`
*   `Q₁₀ = (xᵢ₊₁, yⱼ)`   with value `z[j][i+1]`
*   `Q₀₁ = (xᵢ,   yⱼ₊₁)` with value `z[j+1][i]`
*   `Q₁₁ = (xᵢ₊₁, yⱼ₊₁)` with value `z[j+1][i+1]`

### Step 2: Bilinear Interpolation

Bilinear interpolation estimates the value at point `P` by performing linear interpolation three times.

#### A. The Concept: Linear Interpolation (1D)

First, recall the formula for linear interpolation. Given two points `(x₀, z₀)` and `(x₁, z₁)` and a query point `x` between them, the interpolated value `z` is:

`z = z₀ + (x - x₀) * (z₁ - z₀) / (x₁ - x₀)`

The term `(x - x₀) / (x₁ - x₀)` is the **fractional distance** of `x` along the segment from `x₀` to `x₁`.

#### B. Applying it in 2D (Bilinear)

Bilinear interpolation extends this concept to 2D by treating it as a sequence of 1D interpolations.

The process is as follows:

1.  **Interpolate along the X-axis (bottom edge):**
    Linearly interpolate between `Q₀₀` and `Q₁₀` using the `x` coordinate to find the value at an intermediate point `R₀`.
    `R₀_val = Q₀₀_val + (x - x₀) / (x₁ - x₀) * (Q₁₀_val - Q₀₀_val)`

2.  **Interpolate along the X-axis (top edge):**
    Linearly interpolate between `Q₀₁` and `Q₁₁` using the `x` coordinate to find the value at an intermediate point `R₁`.
    `R₁_val = Q₀₁_val + (x - x₀) / (x₁ - x₀) * (Q₁₁_val - Q₀₁_val)`

3.  **Interpolate along the Y-axis:**
    Finally, linearly interpolate vertically between the results `R₀` and `R₁` using the `y` coordinate to find the final value at `P`.
    `P_val = R₀_val + (y - y₀) / (y₁ - y₀) * (R₁_val - R₀_val)`

This process can be visualized as:

```
      y▲
       │
 y₁ ---Q₀₁ ----------- R₁ ----------- Q₁₁
       │               ▲               │
       │               │ y-interp      │
       │               │               │
  y ---+---------------P---------------+
       │               │               │
       │<-- x-interp -->|<-- x-interp -->|
       │               │               │
 y₀ ---Q₀₀ ----------- R₀ ----------- Q₁₀
       │
       +------------------------------------► x
              x₀               x         x₁
```

### Step 3: Handling Out-of-Bounds Queries (Clamping)

If the query point `(x, y)` lies outside the bounds of the defined grid (e.g., `x < x₀` or `y > y_{NumY-1}`), interpolation is not possible. The implementation handles this with **clamping**.

The `x` and `y` coordinates are clamped to the valid range of their respective axes before any calculations are performed:
*   `x_clamped = std::clamp(x, x_axis.front(), x_axis.back())`
*   `y_clamped = std::clamp(y, y_axis.front(), y_axis.back())`

**Consequence of Clamping:**
*   If `x` is out of bounds, `x_clamped` will be either `x₀` or `x_{NumX-1}`. The fractional distance in the x-direction will become exactly `0` or `1`.
*   If `y` is out of bounds, `y_clamped` will be either `y₀` or `y_{NumY-1}`. The fractional distance in the y-direction will become exactly `0` or `1`.

This effectively means the query is treated as if it were on the nearest edge or corner of the grid, providing a predictable and robust result without requiring special logic for out-of-bounds cases in the core interpolation algorithm.

## 4. Code-to-Math Mapping

Here is how the mathematical variables map to the C++ implementation:

| Math Symbol    | C++ Variable  | Description                                 |
| :------------- | :------------ | :------------------------------------------ |
| `(x, y)`       | `x`, `y`      | The input query coordinates.                |
| `xᵢ`, `yⱼ`     | `x0`, `y0`    | Lower-left corner coordinates of the cell.  |
| `xᵢ₊₁`, `yⱼ₊₁` | `x1`, `y1`    | Upper-right corner coordinates of the cell. |
| `Q₀₀_val`      | `z00`         | Value at `(x0, y0)`.                        |
| `Q₁₀_val`      | `z10`         | Value at `(x1, y0)`.                        |
| `Q₀₁_val`      | `z01`         | Value at `(x0, y1)`.                        |
| `Q₁₁_val`      | `z11`         | Value at `(x1, y1)`.                        |
| `R₀_val`       | `z_interp_y0` | Result of the first x-axis interpolation.   |
| `R₁_val`       | `z_interp_y1` | Result of the second x-axis interpolation.  |
| `P_val`        | `result`      | The final interpolated value.               |