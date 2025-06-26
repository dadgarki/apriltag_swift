// swift-tools-version: 6.1
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "apriltag_swift",
    products: [
        // Products define the executables and libraries a package produces, making them visible to other packages.
        .library(
            name: "apriltag_swift",
            targets: ["apriltag_swift"]),
    ],
    targets: [
        // Targets are the basic building blocks of a package, defining a module or a test suite.
        // Targets can depend on other targets in this package and products from dependencies.
        .target(
            name: "apriltag_c",
            exclude: [
                "apriltag/apriltag_pywrap.c",
                "apriltag/example/opencv_demo.cc",
                "apriltag/example/apriltag_demo.c",
                "apriltag/test/test_detection.c"
            ],
            resources: [
                .copy("apriltag/test/data/")
            ],
            publicHeadersPath: "apriltag"
        ),
        .target(
            name: "apriltag_swift",
            dependencies: ["apriltag_c"]
        ),
        .testTarget(name: "apriltag_swiftTests",
                    dependencies: ["apriltag_swift"]
        )
    ]
)
