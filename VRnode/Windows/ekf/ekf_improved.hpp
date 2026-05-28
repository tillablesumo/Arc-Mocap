#pragma once

#include &lt;array&gt;
#include &lt;span&gt;
#include &lt;cmath&gt;
#include &lt;vector&gt;
#include &lt;algorithm&gt;

// Quaternion structure
struct Quaternion {
    double x, y, z, w;
    
    Quaternion() : x(0), y(0), z(0), w