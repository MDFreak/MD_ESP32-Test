# MD_ESP32-Test

This project is my personal development enirement to implement a general base programming system using arduino framework

General idea:
Different target systems using Arduinos, ESP32 or STM32 controllers should be configurable.
With (maybe non) simple manipulating configuration file for standard demands it should be
possible to start a target system with "plug & connect and configure & play".
This general idea is derived from VSCode using platormIO (does it with SW).
Marlin project realises the same concept generating a configurable firmware for 3d printers

Pathway:
Small projects and experiments are to be realized.
Every project uses a µC and some external components. These will be integrated and selectable

Breaf system description:

- md_lib

This repository is the development repository and uses a collection of other repositories.
This collection is named 'md-lib' and is placed in PIO Projects folder parallel to projects.
Inside your project there is pretty extended file 'prj_config.h'. Using this file you
connect HW with HW (components, pins), HW with SW (libraries, divers etc.).
As well you may conigure SW like tasking, using dual core, using interrupt or polling.

project 'md_lib': test routines to test libraries - somehow a template (to be done)

- md_tools

It containes the file 'md_defines,h', which is the root of all configurations.
In 'md_defines' every configurable component is to be declared.
The library containes some functionalities or glabel usage as for example measurement filtering.

- other libraries of 'md_lib'

Every library inside 'md-lib' is a separate repository. Most of them are implemented as
user interface to the common published driver libraries.
example: WiFi library provides an easy to use scan and login feature for a locale WiFi.