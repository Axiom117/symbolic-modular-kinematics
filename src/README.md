# src

Unified implementation surface for the early-stage repository.

Organize code by domain responsibility, not by target language or milestone:
- encoding -> parse and normalize topology descriptions
- topology -> build and validate graph structure
- templates -> module geometry and joint transform templates
- generator -> derive chains and constraints
- solver -> evaluate FK or IK numerical workflows
- validation and visualization -> support inspection and debugging
